"""
Shared serial reader + CSV logger for the P102 rig on the NUCLEO-G491RE
(firmware P102_LoadDetect_G491RE v4.1). STM copy of bench_common.py.

STM changes: recognises the NUCLEO banner, parses the current view
('# view = x 2') into board_fact('view'), and keeps a per-view board
fact so the dashboard can switch between the 2, 4 and 6 gauge views.

Every sketch in this project emits the same contract:
  - lines beginning with '#'  -> status/comment, surfaced in the log pane
  - everything else           -> comma-separated numeric row, logged verbatim
  - first field is always     -> t_ms (millis since boot)

Commands can be sent back to the board via SerialReader.send().

v3 addition: the reader also PARSES the board's '#' banner for facts the
host needs to cross-check itself against the sketch, principally
countsPerMicrostrain and the bridge config name. Read them with
board_fact(). The '#' lines still go to the log pane unchanged, so nothing
that used this module before needs to change.

v3.6 addition: parses 'rated payload' and 'assist safety factor' into
board_fact('rated_kg') and board_fact('assist_sf').

v3.1 addition: also parses the completion resistor pair, from the
'# comp R2 / R4 = 330 / 330' line, into board_fact('comp_r2') and
board_fact('comp_r4').

Because dtr/rts are held low the Uno does NOT reset when the host
connects, so the boot banner has usually already gone past. The reader
sends 'b' on connect, and from sketch v3.1 'b' reprints the full banner,
so board_fact() fills in within a second or two.
"""

import csv
import re
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial


# Patterns matched against the board's '#' lines.
_FACT_PATTERNS = {
    "counts_per_ustrain": re.compile(r"countsPerMicrostrain\s*=\s*([-\d.]+)"),
    "bridge_factor":      re.compile(r"bridge factor\s*=?\s*([-\d.]+)"),
    "predicted_offset_mv": re.compile(r"predicted off\s*:\s*([-\d.]+)\s*mV"),
    "gauge_ohm":          re.compile(r"gauge / comp\s*=\s*([\d.]+)\s*/"),
    "comp_ohm":           re.compile(r"gauge / comp\s*=\s*[\d.]+\s*/\s*([\d.]+)"),
    "rated_kg":           re.compile(r"rated payload\s*=\s*([\d.]+)\s*kg"),
    "assist_sf":          re.compile(r"assist safety factor\s*=\s*([\d.]+)"),
    # HX711 range -> gain (F407 firmware can run gain 64 for an offset bridge)
    "hx_avdd":            re.compile(r"HX711 AVDD\s*=\s*([\d.]+)\s*V"),
    "hx_fs_mv":           re.compile(r"full scale\s*\+/-\s*([\d.]+)\s*mV"),
}
_COMP_PAIR_RE = re.compile(r"comp R2 / R4\s*=\s*([\d.]+)\s*/\s*([\d.]+)")
_CONFIG_RE = re.compile(r"(?:Uno R3|NUCLEO-G491RE|STM32F407 DISC1) - (\w+)")
_VIEW_RE = re.compile(r"#\s*view\s*=\s*x\s*(\d)")
_GATING_RE = re.compile(r"#\s*pose gating\s*=\s*(ON|OFF)")      # firmware v4.2
_SAVED_SCALE_RE = re.compile(r"saved scale \(view (\d)\)\s*=\s*(none|[-\d.]+)")
_SAVED_OFF_RE = re.compile(r"saved offset\s*=\s*([-\d.]+)\s*counts")
_IMU_RE = re.compile(r"#\s*IMU\s*=\s*(ISM330DHCX|SIMULATED|NOT FOUND)")


class SerialReader:
    """Background serial reader. Parses rows, logs raw CSV, exposes latest state."""

    def __init__(self, port, test_name, field_names, baud=115200, subdir=""):
        self.port = port
        self.baud = baud
        self.field_names = field_names   # names for columns AFTER t_ms
        self.stop_flag = threading.Event()
        self._lock = threading.Lock()
        self._tx_queue = deque()
        self.log_lines = deque(maxlen=400)   # '#' messages from the board
        self._facts = {}                     # parsed from the banner

        data_dir = Path(__file__).resolve().parent.parent / "data"
        if subdir:
            data_dir = data_dir / subdir
        data_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = data_dir / f"{test_name}_{stamp}.csv"
        self.bannername = self.filename.with_suffix(".banner.txt")

        self.state = {"t_ms": 0.0, "rate_hz": 0.0, "connected": False,
                      "last_line_time": time.time()}
        for name in field_names:
            self.state[name] = 0.0

        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def send(self, command):
        """Queue a command to be written to the board (newline appended)."""
        self._tx_queue.append(command)

    def snapshot(self):
        with self._lock:
            return dict(self.state)

    def board_fact(self, key, default=None):
        """A value parsed out of the board's banner, or default if not seen yet."""
        with self._lock:
            return self._facts.get(key, default)

    def forget_fact(self, *keys):
        """Drop facts that are about to change (e.g. after a view switch)."""
        with self._lock:
            for k in keys:
                self._facts.pop(k, None)

    def board_facts(self):
        with self._lock:
            return dict(self._facts)

    def pop_log_lines(self):
        out = []
        while self.log_lines:
            out.append(self.log_lines.popleft())
        return out

    def stop(self):
        self.stop_flag.set()

    # ---------------------------------------------------------- internals
    def _absorb_banner(self, line, banner_fh):
        banner_fh.write(line + "\n")
        banner_fh.flush()
        found = {}
        for key, pat in _FACT_PATTERNS.items():
            m = pat.search(line)
            if m:
                try:
                    found[key] = float(m.group(1))
                except ValueError:
                    pass
        m = _COMP_PAIR_RE.search(line)
        if m:
            try:
                found["comp_r2"] = float(m.group(1))
                found["comp_r4"] = float(m.group(2))
            except ValueError:
                pass
        m = _CONFIG_RE.search(line)
        if m:
            found["config"] = m.group(1)
        m = _VIEW_RE.search(line)
        if m:
            found["view"] = int(m.group(1))
        m = _GATING_RE.search(line)
        if m:
            found["pose_gating"] = m.group(1) == "ON"
        m = _SAVED_SCALE_RE.search(line)
        if m:                          # print_cal: scale first, then (if any) offset
            found["saved_scale_view"] = int(m.group(1))
            found["saved_scale"] = None if m.group(2) == "none" else float(m.group(2))
            found["saved_offset"] = None
        m = _SAVED_OFF_RE.search(line)
        if m:
            found["saved_offset"] = float(m.group(1))
        m = _IMU_RE.search(line)
        if m:
            found["imu_found"] = m.group(1) != "NOT FOUND"
        if found:
            with self._lock:
                self._facts.update(found)

    def _run(self):
        try:
            # serial_for_url: a COM port name, or a URL such as
            # socket://localhost:9090 (F407 Discovery: RTT over the ST-Link,
            # served by tools/rtt_server.sh - its USB COM port isn't wired).
            ser = serial.serial_for_url(self.port, do_not_open=True)
            ser.baudrate = self.baud
            ser.timeout = 1
            ser.dtr = False          # don't glitch-reset the board on connect
            ser.rts = False
            ser.open()
        except serial.SerialException as e:
            self.log_lines.append(f"# ERROR: could not open {self.port}: {e}")
            self.stop_flag.set()
            return

        time.sleep(2)                # let the board finish booting
        ser.reset_input_buffer()

        # RotaTruck firmware: the bench-rig commands and CSV live inside the
        # load-detect console. 'LD 1' enters it (assist forced off); harmless
        # on the stand-alone rig firmware (unknown command -> command list).
        ser.write(b"\nLD 1\n")
        time.sleep(0.3)

        # Ask the board to reprint its banner. Harmless on sketches that do
        # not know 'b': they just print their command list.
        ser.write(b"b\n")

        expected = 1 + len(self.field_names)
        sample_count = 0
        last_rate_calc = time.time()

        with open(self.filename, "w", newline="") as f, \
             open(self.bannername, "w") as bf:
            writer = csv.writer(f)
            writer.writerow(["t_ms"] + self.field_names)

            while not self.stop_flag.is_set():
                while self._tx_queue:
                    cmd = self._tx_queue.popleft()
                    ser.write((cmd + "\n").encode())
                    self.log_lines.append(f">> sent: {cmd}")

                if ser.in_waiting:
                    line = ser.readline().decode(errors="replace").strip()
                    if not line:
                        continue

                    if line.startswith("#"):
                        self.log_lines.append(line)
                        self._absorb_banner(line, bf)
                        continue

                    parts = line.split(",")
                    if len(parts) > expected or len(parts) < 2:
                        continue
                    try:
                        values = [float(p) for p in parts]
                    except ValueError:
                        continue
                    # an older sketch sends fewer columns: pad with NaN
                    while len(values) < expected:
                        values.append(float("nan"))
                        parts.append("")

                    writer.writerow(parts)
                    f.flush()
                    sample_count += 1

                    with self._lock:
                        self.state["t_ms"] = values[0]
                        for name, val in zip(self.field_names, values[1:]):
                            self.state[name] = val
                        self.state["connected"] = True
                        self.state["last_line_time"] = time.time()

                if time.time() - last_rate_calc >= 1.0:
                    with self._lock:
                        self.state["rate_hz"] = sample_count / (time.time() - last_rate_calc)
                    sample_count = 0
                    last_rate_calc = time.time()

                time.sleep(0.001)

        try:
            ser.write(b"LD 0\n")     # RotaTruck: back to the motor console
            time.sleep(0.2)
        except serial.SerialException:
            pass
        ser.close()


def connection_status(snap):
    """Returns (label, colour) for the status line."""
    age = time.time() - snap["last_line_time"]
    if snap["connected"] and age < 1.5:
        return "LIVE", "black"
    return "NO DATA / STALLED", "red"


def linear_fit(xs, ys):
    """Least-squares fit y = m*x + c. Returns (m, c, r_squared) or (None,)*3."""
    n = len(xs)
    if n < 2:
        return None, None, None
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx == 0:
        return None, None, None
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    m = sxy / sxx
    c = mean_y - m * mean_x
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (m * x + c)) ** 2 for x, y in zip(xs, ys))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return m, c, r2

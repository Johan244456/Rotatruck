"""
Middle strip (CH1, 2 SG) raw-output check for the RotaTruck load detection.

Shows the middle strip's HX711 output live:
  - ZEROED: counts / mV relative to a zero, so the bridge's permanent resting
    offset (completion-resistor mismatch) is removed and the reading starts
    at 0. The window zeroes itself on connect; "Zero here" re-zeroes;
    "Save zero to board (z)" stores the zero in the firmware's flash so the
    board's own readings (dashboard, kg) start from it too.
  - RAW: the untared 24-bit counts and % of the HX711's full scale, which is
    what decides railing (an offset can be tared, a railed input cannot).

One HX711 is enough: the board is switched to the middle-only view (x 1) on
connect and the previous view is restored on close. The HX711 range (gain)
is read from the firmware's banner, so the mV figures always match it.

Firmware: env:loadcell_test (or the full firmware with the LD console).

Run (F407 Discovery, console over RTT):
  terminal 1:  tools/rtt_server.sh
  terminal 2:  python3 middle_raw_check.py                 # window
  Every sample is logged to data/strain_gauge/rawcheck_<date>_<time>.csv.
               python3 middle_raw_check.py --cli 20        # text only, 20 s
  Other port:  python3 middle_raw_check.py /dev/ttyUSB0    # needs pyserial

Verdict (from the RAW counts):
  OK           inside the HX711 range.
  NEAR RAIL    above 90 % of full scale.
  RAILED       |raw| >= 8 300 000 (firmware threshold): |A+ - A-| beyond the
               range - bridge out of balance, arm open, or A+/A- floating.
  FROZEN       20 identical samples in a row.
  NO DATA      the HX711 is not converting (power, DT -> PE11, SCK -> PE12).
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import queue
import re
import socket
import statistics
import sys
import threading
import time
from collections import deque

DEFAULT_PORT = "socket://localhost:9090"
RAIL = 8388607                 # HX711 full scale, counts
RAIL_WARN = 8300000            # firmware's RAILED threshold (loadcell.c HX_RAIL_WARN)
NEAR_FRAC = 0.90
FROZEN_N = 20
HX_AVDD = 2.9                  # V, DISC1 3V rail (loadcell.c HX_AVDD)
STATS_N = 50                   # samples (5 s at 10 Hz) for mean / sd / p-p
ZERO_N = 20                    # samples averaged for a zero (2 s)
PLOT_N = 300                   # samples on the plot (30 s)
LOG_DIR = Path(__file__).resolve().parent.parent / "data" / "strain_gauge"


def fs_mv_for_gain(gain):
    return 0.5 * HX_AVDD / gain * 1000.0


# ------------------------------------------------------------------ transport
class Link:
    """Line-based link to the board: raw TCP for socket://, pyserial otherwise."""

    def __init__(self, port):
        self.sock = None
        self.ser = None
        self.lock = threading.Lock()
        if port.startswith("socket://"):
            host, _, p = port[len("socket://"):].partition(":")
            self.sock = socket.create_connection((host or "localhost", int(p or 9090)), 3)
            self.sock.settimeout(0.2)
        else:
            import serial          # only needed for real serial ports
            self.ser = serial.Serial(port, 115200, timeout=0.2)

    def read(self):
        try:
            if self.sock:
                d = self.sock.recv(4096)
                if not d:
                    raise ConnectionError("connection closed")
                return d
            return self.ser.read(4096)
        except socket.timeout:
            return b""

    def send(self, line):
        with self.lock:
            data = line.encode() + b"\r\n"
            if self.sock:
                self.sock.sendall(data)
            else:
                self.ser.write(data)

    def close(self):
        try:
            if self.sock:
                self.sock.close()
            if self.ser:
                self.ser.close()
        except OSError:
            pass


# ------------------------------------------------------------------ model
class RawCheck:
    """Reader thread + middle-strip state. GUI and CLI both use this."""

    def __init__(self, port, gain=None):
        self.link = Link(port)
        self.lines = queue.Queue()          # '#' lines for the log
        self.raw = deque(maxlen=PLOT_N)     # (t_s, raw) of real samples
        self.last_raw = None
        self.same = 0
        self.nodata = False
        self.last_row_t = 0.0
        self.t0 = time.time()
        self.orig_view = None
        self.zero = None                    # raw counts of the zero (None until taken)
        self.zero_pending = True            # auto-zero once ZERO_N samples are in
        self.fs_mv = fs_mv_for_gain(gain or 128)
        self.fs_from_board = False
        self.gain_forced = gain is not None
        self.stop = threading.Event()
        self._buf = b""
        self._view_seen = threading.Event()
        # every sample -> data/strain_gauge/rawcheck_<date>_<time>.csv
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_DIR / f"rawcheck_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self._log_fh = open(self.log_path, "w", newline="", buffering=1)
        self._log = csv.writer(self._log_fh)
        self._log.writerow(["time", "t_s", "raw_counts", "zero_counts", "zeroed_counts",
                            "zeroed_uV", "input_mV", "pct_full_scale", "hx711_fs_mV"])
        self.lines.put(f"# GUI: logging every sample to {self.log_path}")
        threading.Thread(target=self._run, daemon=True).start()

    # -- setup / teardown
    def start(self):
        self.link.send("LD 1")              # console on (no-op if already on)
        time.sleep(0.3)
        self.link.send("b")                 # banner first -> HX711 full scale (gain) before the auto-zero
        time.sleep(0.3)
        self.link.send("v")                 # report current view -> orig_view
        self._view_seen.wait(2.0)
        if self.orig_view != 1:
            self.lines.put(f"# GUI: switching to middle-only view (x 1); "
                           f"x {self.orig_view if self.orig_view is not None else 0} restored on close")
            self.link.send("x 1")

    def close(self):
        try:
            if self.orig_view not in (None, 1):
                self.link.send(f"x {self.orig_view}")
                time.sleep(0.3)
        except OSError:
            pass
        self.stop.set()
        self.link.close()
        try:
            self._log_fh.close()
        except OSError:
            pass

    # -- zero
    def zero_here(self):
        """Zero the display on the mean of the last ZERO_N samples."""
        vals = [r for _, r in list(self.raw)[-ZERO_N:]]
        if len(vals) < 5:
            self.zero_pending = True        # take it as soon as there are samples
            return False
        self.zero = statistics.fmean(vals)
        self.zero_pending = False
        self.lines.put(f"# GUI: zero = {self.zero:+.0f} raw counts "
                       f"({self.counts_to_mv(self.zero):+.4f} mV resting offset, now shown as 0)")
        return True

    def save_zero_to_board(self):
        """'z' = the firmware saves the current reading as its zero (flash)."""
        self.link.send("z")
        self.zero_here()

    # -- reader
    def _run(self):
        while not self.stop.is_set():
            try:
                d = self.link.read()
            except (OSError, ConnectionError) as e:
                self.lines.put(f"# GUI: link lost ({e})")
                self.stop.set()
                return
            if not d:
                continue
            self._buf += d
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                self._line(line.decode(errors="replace").strip())

    def _line(self, s):
        if not s:
            return
        if s.startswith("#") or not s[0].isdigit():
            m = re.search(r"view x (\d)", s)
            if m and self.orig_view is None:
                self.orig_view = int(m.group(1))
                self._view_seen.set()
            m = re.search(r"full scale \+/-([\d.]+) mV", s)
            if m and not self.gain_forced:
                self.fs_mv = float(m.group(1))
                if not self.fs_from_board:
                    self.fs_from_board = True
                    gain = round(0.5 * HX_AVDD * 1000.0 / self.fs_mv)
                    self.lines.put(f"# GUI: board HX711 range +/-{self.fs_mv:.2f} mV (gain {gain})")
            if not s.startswith("t_ms"):
                self.lines.put(s)
            return
        f = s.split(",")
        if len(f) < 9:
            return
        try:
            raw = int(float(f[1]))
            status = int(f[8])
        except ValueError:
            return
        self.last_row_t = time.time()
        if status == 5:                      # NO_DATA rows carry zeros, not samples
            self.nodata = True
            return
        self.nodata = False
        self.same = self.same + 1 if raw == self.last_raw else 0
        self.last_raw = raw
        self.raw.append((time.time() - self.t0, raw))
        z = self.zeroed(raw)
        try:
            self._log.writerow([datetime.now().isoformat(timespec="milliseconds"),
                                f"{time.time() - self.t0:.2f}", raw,
                                "" if self.zero is None else f"{self.zero:.0f}",
                                "" if z is None else f"{z:.0f}",
                                "" if z is None else f"{self.counts_to_mv(z) * 1000:.2f}",
                                f"{self.counts_to_mv(raw):.5f}", f"{abs(raw) / RAIL * 100:.3f}",
                                f"{self.fs_mv:.2f}"])
        except (OSError, ValueError):
            pass
        if (self.zero_pending and len(self.raw) >= ZERO_N and abs(raw) < RAIL_WARN
                and (self.fs_from_board or self.gain_forced or time.time() - self.t0 > 5.0)):
            self.zero_here()

    # -- numbers
    def counts_to_mv(self, c):
        return c / RAIL * self.fs_mv

    def zeroed(self, raw):
        return raw - self.zero if self.zero is not None else None

    def verdict(self):
        """(state, colour, detail) for the current reading (from RAW counts)."""
        if time.time() - self.last_row_t > 2.0:
            return "NO STREAM", "#888888", "no rows from the board (RTT server running? firmware?)"
        if self.nodata or self.last_raw is None:
            return "NO DATA", "#c0392b", "HX711 not converting: check VCC/GND, DT -> PE11, SCK -> PE12"
        r = self.last_raw
        if abs(r) >= RAIL_WARN:
            side = "+" if r > 0 else "-"
            return "RAILED", "#c0392b", (f"stuck at {side}full scale: |A+ - A-| > {self.fs_mv:.1f} mV - bridge "
                                         "out of balance (resistor mismatch / bad contact), arm open, "
                                         "or A+/A- floating. A zero can't fix this - it needs range.")
        if self.same >= FROZEN_N:
            return "FROZEN", "#c0392b", f"{self.same + 1} identical samples"
        if abs(r) >= NEAR_FRAC * RAIL:
            return "NEAR RAIL", "#d68910", "over 90 % of full scale: little headroom left for load"
        head = (1 - abs(r) / RAIL) * self.fs_mv
        return "OK", "#1e8449", f"inside the HX711 range ({head:.1f} mV headroom before the rail)"

    def stats(self):
        vals = [r for _, r in list(self.raw)[-STATS_N:]]
        if len(vals) < 2:
            return None
        return {"n": len(vals), "mean": statistics.fmean(vals), "min": min(vals),
                "max": max(vals), "pp": max(vals) - min(vals), "sd": statistics.pstdev(vals)}


# ------------------------------------------------------------------ CLI
def run_cli(port, secs, gain):
    rc = RawCheck(port, gain)
    rc.start()
    end = time.time() + secs
    try:
        while time.time() < end and not rc.stop.is_set():
            time.sleep(1.0)
            while not rc.lines.empty():
                print(rc.lines.get())
            state, _, detail = rc.verdict()
            r = rc.last_raw
            if r is None:
                print(f"{state:9s} [{detail}]", flush=True)
                continue
            z = rc.zeroed(r)
            zs = (f"zeroed {z:+9.0f} cts {rc.counts_to_mv(z) * 1000:+9.1f} uV" if z is not None
                  else "zeroed  (waiting for samples)")
            line = (f"{state:9s} {zs} | raw {r:+9d} {abs(r) / RAIL * 100:5.1f}% FS "
                    f"{rc.counts_to_mv(r):+8.3f} mV")
            st = rc.stats()
            if st:
                line += f" | sd {st['sd']:.0f} cts = {rc.counts_to_mv(st['sd']) * 1000:.2f} uV"
            print(f"{line}  [{detail}]", flush=True)
    finally:
        rc.close()


# ------------------------------------------------------------------ GUI
def run_gui(port, gain):
    import tkinter as tk
    from tkinter import ttk, messagebox
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    root = tk.Tk()
    root.title("Middle strip (CH1, 2 SG) - HX711 check")
    root.geometry("1040x760")

    state = {"rc": None}

    top = ttk.Frame(root, padding=8)
    top.pack(fill=tk.X)
    ttk.Label(top, text="Port").pack(side=tk.LEFT)
    port_var = tk.StringVar(value=port)
    ttk.Entry(top, textvariable=port_var, width=28).pack(side=tk.LEFT, padx=6)
    conn_btn = ttk.Button(top, text="Connect")
    conn_btn.pack(side=tk.LEFT)
    ttk.Label(top, text="   (run tools/rtt_server.sh first; close the PlatformIO monitor)",
              foreground="#666").pack(side=tk.LEFT)

    # verdict + numbers
    vf = ttk.Frame(root, padding=(8, 0))
    vf.pack(fill=tk.X)
    verdict_lbl = tk.Label(vf, text="NOT CONNECTED", font=("TkDefaultFont", 24, "bold"),
                           fg="white", bg="#888888", width=13, pady=6)
    verdict_lbl.pack(side=tk.LEFT, padx=(0, 12))
    zero_var = tk.StringVar(value="ZEROED\n-")
    tk.Label(vf, textvariable=zero_var, font=("TkFixedFont", 16, "bold"), justify=tk.LEFT,
             fg="#1a5276").pack(side=tk.LEFT, padx=(0, 16))
    num_var = tk.StringVar(value="raw  -\n% FS -\nmV   -")
    tk.Label(vf, textvariable=num_var, font=("TkFixedFont", 11), justify=tk.LEFT).pack(side=tk.LEFT)
    stat_var = tk.StringVar(value="")
    tk.Label(vf, textvariable=stat_var, font=("TkFixedFont", 10), justify=tk.LEFT,
             fg="#333").pack(side=tk.LEFT, padx=16)
    detail_var = tk.StringVar(value="")
    ttk.Label(root, textvariable=detail_var, padding=(10, 4)).pack(fill=tk.X)

    # zero buttons
    zf = ttk.Frame(root, padding=(8, 0))
    zf.pack(fill=tk.X)

    def zero_here():
        rc = state["rc"]
        if rc and not rc.zero_here():
            messagebox.showinfo("Zero", "Not enough samples yet - it will zero as soon as they arrive.")

    def save_zero():
        rc = state["rc"]
        if not rc:
            return
        if messagebox.askyesno(
                "Save zero to board",
                "Save the current reading as the board's zero (flash)?\n\n"
                "Plate empty, hands off (on the truck: tipped back). The board pauses "
                "~1-2 s while it saves."):
            rc.save_zero_to_board()

    ttk.Button(zf, text="Zero here", command=zero_here).pack(side=tk.LEFT, padx=(0, 6))
    ttk.Button(zf, text="Save zero to board (z)", command=save_zero).pack(side=tk.LEFT)
    ttk.Label(zf, text="   The resting offset is removed from the ZEROED reading; "
                       "railing is judged on the raw counts.", foreground="#666").pack(side=tk.LEFT)

    # plot
    fig = Figure(figsize=(9.5, 3.6), dpi=100)
    ax = fig.add_subplot(111)
    ax.set_xlabel("time (s)")
    (line,) = ax.plot([], [], lw=1.4, color="#1f77b4")
    rail_lines = []
    fig.tight_layout()
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8)
    plot_var = tk.StringVar(value="zeroed")

    # commands + log
    cf = ttk.Frame(root, padding=8)
    cf.pack(fill=tk.X)
    ttk.Label(cf, text="Plot:").pack(side=tk.LEFT)
    ttk.Radiobutton(cf, text="zeroed (uV)", value="zeroed", variable=plot_var).pack(side=tk.LEFT)
    ttk.Radiobutton(cf, text="raw, full HX711 range", value="full", variable=plot_var).pack(side=tk.LEFT)
    ttk.Label(cf, text="    Board:").pack(side=tk.LEFT)

    def send(cmd):
        rc = state["rc"]
        if rc:
            try:
                rc.link.send(cmd)
            except OSError as e:
                messagebox.showerror("Send failed", str(e))

    ttk.Button(cf, text="Diagnose (d)", command=lambda: send("d")).pack(side=tk.LEFT, padx=2)
    ttk.Button(cf, text="Channels (v)", command=lambda: send("v")).pack(side=tk.LEFT, padx=2)
    cmd_var = tk.StringVar()
    ent = ttk.Entry(cf, textvariable=cmd_var, width=16)
    ent.pack(side=tk.LEFT, padx=(10, 2))

    def send_box(_e=None):
        if cmd_var.get().strip():
            send(cmd_var.get().strip())
            cmd_var.set("")
    ent.bind("<Return>", send_box)
    ttk.Button(cf, text="Send", command=send_box).pack(side=tk.LEFT)

    log = tk.Text(root, height=8, font=("TkFixedFont", 9))
    log.pack(fill=tk.BOTH, padx=8, pady=(0, 8))

    def log_add(s):
        log.insert(tk.END, s + "\n")
        log.see(tk.END)

    def connect():
        if state["rc"]:
            state["rc"].close()
            state["rc"] = None
            conn_btn.config(text="Connect")
            verdict_lbl.config(text="NOT CONNECTED", bg="#888888")
            return
        try:
            rc = RawCheck(port_var.get().strip(), gain)
        except Exception as e:          # noqa: BLE001 - show any connect error
            messagebox.showerror("Connect failed",
                                 f"{e}\n\nIs tools/rtt_server.sh running, and the PlatformIO monitor closed?")
            return
        state["rc"] = rc
        conn_btn.config(text="Disconnect")
        threading.Thread(target=rc.start, daemon=True).start()

    conn_btn.config(command=connect)

    def tick():
        rc = state["rc"]
        if rc:
            while not rc.lines.empty():
                log_add(rc.lines.get())
            st_, colour, detail = rc.verdict()
            verdict_lbl.config(text=st_, bg=colour)
            detail_var.set(detail)
            r = rc.last_raw
            if r is not None:
                num_var.set(f"raw  {r:+9d}\n% FS {abs(r) / RAIL * 100:7.2f} %\n"
                            f"mV   {rc.counts_to_mv(r):+9.4f}  (range +/-{rc.fs_mv:.1f})")
                z = rc.zeroed(r)
                zero_var.set("ZEROED\n" + (f"{z:+9.0f} cts\n{rc.counts_to_mv(z) * 1000:+9.1f} uV"
                                           if z is not None else "(zeroing...)"))
            s = rc.stats()
            if s:
                stat_var.set(f"last {s['n']} samples\n"
                             f"p-p {s['pp']} cts = {rc.counts_to_mv(s['pp']) * 1000:.1f} uV\n"
                             f"sd  {s['sd']:.0f} cts = {rc.counts_to_mv(s['sd']) * 1000:.2f} uV")
            pts = list(rc.raw)
            if pts:
                xs = [p[0] for p in pts]
                for ln in rail_lines:
                    ln.remove()
                rail_lines.clear()
                if plot_var.get() == "full":
                    ys = [p[1] for p in pts]
                    ax.set_ylabel("raw counts")
                    ax.set_ylim(-RAIL * 1.05, RAIL * 1.05)
                    for y, c, ls in ((RAIL, "#c0392b", "-"), (-RAIL, "#c0392b", "-"),
                                     (NEAR_FRAC * RAIL, "#d68910", "--"),
                                     (-NEAR_FRAC * RAIL, "#d68910", "--")):
                        rail_lines.append(ax.axhline(y, color=c, ls=ls, lw=1))
                else:
                    z0 = rc.zero if rc.zero is not None else pts[0][1]
                    ys = [rc.counts_to_mv(p[1] - z0) * 1000 for p in pts]
                    ax.set_ylabel("from zero (uV)")
                    lo, hi = min(ys), max(ys)
                    pad = max((hi - lo) * 0.15, 5.0)
                    ax.set_ylim(lo - pad, hi + pad)
                    rail_lines.append(ax.axhline(0, color="#999999", ls=":", lw=1))
                line.set_data(xs, ys)
                ax.set_xlim(xs[0], max(xs[-1], xs[0] + 1))
                canvas.draw_idle()
        root.after(200, tick)

    def on_close():
        if state["rc"]:
            state["rc"].close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(200, tick)
    connect()                            # try straight away
    root.mainloop()


def main():
    ap = argparse.ArgumentParser(description="Middle strip (CH1) HX711 check")
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT,
                    help=f"default {DEFAULT_PORT} (tools/rtt_server.sh)")
    ap.add_argument("--cli", type=float, metavar="SECS",
                    help="no window: print once a second for SECS seconds")
    ap.add_argument("--gain", type=int, choices=(128, 64),
                    help="override the HX711 gain (normally read from the board)")
    a = ap.parse_args()
    if a.cli:
        try:
            run_cli(a.port, a.cli, a.gain)
        except OSError as e:
            sys.exit(f"could not connect to {a.port}: {e}\nStart tools/rtt_server.sh first.")
    else:
        run_gui(a.port, a.gain)


if __name__ == "__main__":
    main()

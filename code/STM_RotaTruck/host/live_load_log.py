"""
RotaTruck live status - load detection + assist, in plain language.

Shows what the trolley is doing right now (upright / tipped back, load on the
plate, the load the assist is set for, handle, assist on/off) and announces
what happens (load accepted, scale zeroed, assist on/off, problems). The full
technical data is still logged to data/strain_gauge/live_load_<date>_<time>.csv.

Run (with the board on USB):
  terminal 1:  tools/rtt_server.sh --attach      (or have the PlatformIO serial monitor open)
  terminal 2:  python3 host/live_load_log.py               Ctrl-C to stop
               python3 host/live_load_log.py --tech        engineer view (every value, 5 per second)

How to use the handle:
  1 click  while UPRIGHT (hands off the load for ~1 s)  -> accept the load
  3 clicks while TIPPED BACK, plate empty                -> zero the scale
  hold                                                   -> assist on (release = stop)
The assist is set for 0.7 x a MEASURED load (never more than the real load), and
1.0 x a MANUAL load (the 15 kg power-up default or 'W <kg>').
"""

import argparse
import csv
import re
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent.parent / "data" / "strain_gauge"
FIELDS = ["t_ms", "handle", "pitch_deg", "lc_pose", "stand_dev_deg", "rest_dev_deg", "load_status",
          "measured_kg", "offer_kg", "load_kg", "payload_kg", "payload_src", "assist_on", "assist_pose"]
EVENT_PREFIXES = ("SAFETY:", "PAYLOAD:", "HANDLE:", "ASSIST:", "Pose:", "# LOAD", "DRIVING", "STANDING",
                  "IMU:", "ERR", "AUTO:", "MODE:", "DRIVE:")
ACCEPT_MAX_DEV = 2.0          # board.h PAYLOAD_ACCEPT_MAX_DEV_DEG

TTY = sys.stdout.isatty()
def col(s, c):
    codes = {"g": "32", "r": "31", "y": "33", "c": "36", "b": "1", "d": "2"}
    return f"\033[{codes[c]}m{s}\033[0m" if TTY else s


# ------------------------------------------------------------------ plain-language status
def pose_text(r):
    p, dev = r["lc_pose"], float(r["stand_dev_deg"])
    if p == "NO IMU":
        return col("tilt sensor not working", "r")
    if p == "STANDING":
        if dev <= ACCEPT_MAX_DEV:
            return col("UPRIGHT - ready to accept load", "g")
        return col(f"upright, {dev:.0f} deg off - stand it fully up", "y")
    if p.startswith("TIPPED BACK") or p == "TILTING":
        if r.get("assist_pose") == "DRIVING":
            return col("FULLY TIPPED BACK - assist can drive", "c")
        return col(f"tipped back {float(r['rest_dev_deg']):.0f} deg short of fully - wheels held", "y")
    if p == "TILTING":
        return col("tilting...", "y")
    return p.lower()


def load_text(r):
    st, kg = r["load_status"], float(r["measured_kg"])
    if st in ("OK", "OVERLOAD"):
        s = f"{kg:5.1f} kg"
        return col(s + (" (over rated load!)" if st == "OVERLOAD" else ""), "g" if st == "OK" else "r")
    if st == "SETTLING":
        return col(f"{kg:5.1f} kg (settling...)", "y")
    if st == "HELD":
        return col("held while tipped back", "d")
    if st == "NO_CAL":
        return col("scale not calibrated", "r")
    if st == "IMU_FAULT":
        return col(f"{kg:5.1f} kg (tilt sensor needed)", "r")
    return col(f"{kg:5.1f} kg (not used: {st.lower()})", "r")     # always show the actual value


def status_line(r):
    assist_kg = float(r["payload_kg"])
    load_kg = float(r["load_kg"])
    src = "accepted" if r["payload_src"] == "ACCEPTED" else "default"
    handle = col("HELD", "g") if r["handle"] == "1" else "released"
    assist = col("ON", "c") if r["assist_on"] == "1" else "off"
    return (f"Trolley: {pose_text(r)}  |  On plate: {load_text(r)}  |  "
            f"Assist set for {assist_kg:.1f} kg ({src} load {load_kg:.1f} kg)  |  Handle: {handle}  |  Assist: {assist}")


# ------------------------------------------------------------------ plain-language events
_EVENTS = [
    (r"PAYLOAD: ACCEPTED - load initialised ([\d.]+) kg -> assist payload ([\d.]+) kg",
     lambda m: col(f"[OK] Load accepted: {m[1]} kg  ->  assist will help with {m[2]} kg", "g")),
    (r"PAYLOAD: load (?:detected|updated) ([\d.]+) kg \(upright, auto\) -> assist payload ([\d.]+) kg",
     lambda m: col(f"[OK] Load read: {m[1]} kg  ->  assist will help with {m[2]} kg", "g")),
    (r"not exactly at the standing angle \(([\d.]+) deg",
     lambda m: col(f"[X] Load NOT accepted - trolley is {float(m[1]):.0f} deg from upright. Stand it fully up and click once.", "r")),
    (r"no settled reading",
     lambda m: col("[X] Load NOT accepted - keep hands off the load for about 1 second, then click once.", "r")),
    (r"PAYLOAD: not accepted - no IMU",
     lambda m: col("[X] Load NOT accepted - the tilt sensor is not working.", "r")),
    (r"PAYLOAD: scale zeroed - load back to MANUAL ([\d.]+) kg",
     lambda m: col(f"[i] Load back to manual {m[1]} kg (scale zeroed).", "y")),
    (r"ZEROED by handle triple-click",
     lambda m: col("[OK] Scale zeroed (empty).", "g")),
    (r"handle zero refused - not tipped back",
     lambda m: col("[X] Zero refused - tip the trolley back (plate empty), then triple-click.", "r")),
    (r"handle zero refused - no IMU",
     lambda m: col("[X] Zero refused - the tilt sensor is not working.", "r")),
    (r"LOAD: rest zero",
     lambda m: col("[i] Scale re-zeroed itself while resting tipped back.", "d")),
    (r"HANDLE: 2 quick clicks",
     lambda m: col("[i] Two clicks do nothing.  1 click (upright) = accept load, 3 clicks (tipped back) = zero, hold = assist.", "y")),
    (r"ASSIST: ENABLED",
     lambda m: col(">> Assist ON - tip back to drive; release the handle to stop.", "c")),
    (r"ASSIST: cannot enable",
     lambda m: col("[X] Assist could not start - the motors are not powered / not connected.", "r")),
    (r"ASSIST: disabled",
     lambda m: col("[] Assist OFF (motors idle).", "c")),
    (r"Pose: motors BRAKED",
     lambda m: col("   Wheels held (not fully tipped back).", "d")),
    (r"Pose: motors RELEASED",
     lambda m: col("   Fully tipped back - assist driving the wheels.", "d")),
    (r"handle released - ramping to a stop",
     lambda m: col("   Handle released - slowing to a stop.", "c")),
    (r"LOAD: IMU lost",
     lambda m: col("[!] Tilt sensor lost - load held at 0, wheels held.", "r")),
    (r"LOAD: IMU back",
     lambda m: col("[OK] Tilt sensor back.", "g")),
    (r"IMU: ISM330DHCX configured \(recovered\)",
     lambda m: col("[OK] Tilt sensor reconnected.", "g")),
    (r"# LOAD (NO_DATA|RAILED|FROZEN|IMPLAUSIBLE)",
     lambda m: col("[!] Load sensor problem - reading not trusted (check wiring, then zero).", "r")),
    (r"RotaTruck ODrive-S1 firmware online",
     lambda m: col("== Controller restarted ==", "b")),
]
_last_event = {"text": "", "t": 0.0}


def event_text(line):
    for pat, fn in _EVENTS:
        m = re.search(pat, line)
        if m:
            return fn(m)
    return None


# ------------------------------------------------------------------ transport
def connect(port):
    host, _, p = port.partition(":")
    s = socket.create_connection((host or "localhost", int(p or 9090)), 3)
    s.settimeout(0.3)
    return s


def fmt_row_tech(r):
    return (f"{float(r['t_ms'])/1000:8.1f}s  handle {r['handle']}  pitch {float(r['pitch_deg']):+6.1f}  "
            f"pose {r['lc_pose'][:12]:12s} stand {float(r['stand_dev_deg']):5.1f} rest {float(r['rest_dev_deg']):5.1f}deg  load {r['load_status']:11s} "
            f"meas {float(r['measured_kg']):7.1f} kg | load {float(r['load_kg']):6.1f} ({r['payload_src']}) "
            f"-> assist {float(r['payload_kg']):5.1f} kg  assist {r['assist_on']} {r['assist_pose']}")


def main():
    ap = argparse.ArgumentParser(description="RotaTruck live status (load detection + assist)")
    ap.add_argument("--port", default="localhost:9090", help="RTT server host:port (default localhost:9090)")
    ap.add_argument("--tech", action="store_true", help="engineer view: every row and every board message")
    a = ap.parse_args()

    try:
        s = connect(a.port)
    except OSError as e:
        sys.exit(f"Can't reach the controller ({e}).\n"
                 "Start the link first:  tools/rtt_server.sh --attach   (or open the PlatformIO serial monitor)")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = LOG_DIR / f"live_load_{datetime.now():%Y%m%d_%H%M%S}.csv"
    fh = open(path, "w", newline="", buffering=1)
    w = csv.writer(fh)
    w.writerow(["time"] + FIELDS + ["event"])
    print(col("RotaTruck live status", "b") + f"   (full data logged to {path.name})")
    print(col("Handle: 1 click upright = accept load  |  3 clicks tipped back = zero  |  hold = assist on.  Ctrl-C to stop.\n", "d"))

    s.sendall(b"\r\nPL 1\r\n")
    buf, n, last_row, last_status, status_shown = b"", 0, time.time(), "", 0.0
    try:
        while True:
            try:
                d = s.recv(4096)
                if not d:
                    print(col("\n[!] Lost the link to the controller (RTT server stopped or the USB dropped).", "r"))
                    break
                buf += d
            except socket.timeout:
                pass
            if time.time() - last_row > 3.0:
                if not a.tech and TTY:
                    print()
                print(col("[!] No data from the controller for 3 s - retrying...", "y"))
                s.sendall(b"PL 1\r\n")
                last_row = time.time()
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                t = line.decode(errors="replace").strip()
                if not t:
                    continue
                now = datetime.now().isoformat(timespec="milliseconds")
                if t.startswith("PL,"):
                    parts = t.split(",")[1:]
                    if len(parts) != len(FIELDS):
                        continue
                    r = dict(zip(FIELDS, parts))
                    w.writerow([now] + parts + [""])
                    last_row = time.time()
                    n += 1
                    try:
                        if a.tech:
                            print(fmt_row_tech(r))
                        else:
                            line_txt = status_line(r)
                            if TTY:     # one live line, updated in place
                                sys.stdout.write("\r\033[K" + line_txt)
                                sys.stdout.flush()
                                status_shown = time.time()
                            elif line_txt != last_status or time.time() - status_shown > 5:
                                print(line_txt)
                                status_shown = time.time()
                            last_status = line_txt
                    except ValueError:
                        pass
                elif t.startswith(EVENT_PREFIXES) or "firmware online" in t:
                    w.writerow([now] + [""] * len(FIELDS) + [t])
                    if a.tech:
                        print(col(f"  >>> {t}", "c"))
                        continue
                    msg = event_text(t)
                    if msg and not (msg == _last_event["text"] and time.time() - _last_event["t"] < 2):
                        _last_event.update(text=msg, t=time.time())
                        stamp = datetime.now().strftime("%H:%M:%S")
                        if TTY:
                            sys.stdout.write("\r\033[K")
                        print(f"{stamp}  {msg}")
                        if TTY and last_status:
                            sys.stdout.write(last_status)
                            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            s.sendall(b"PL 0\r\n")
        except OSError:
            pass
        s.close()
        fh.close()
        print(f"\n\nStopped.  {n} readings saved to {path}")


if __name__ == "__main__":
    main()

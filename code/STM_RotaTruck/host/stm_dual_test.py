"""
Dual-bridge drift and stability test GUI for the NUCLEO-G491RE (firmware v4.1).
STM copy of dual_bridge_test.py.

Run it: double-click, Run in VS Code, or  python stm_dual_test.py
A start window asks for the port, test name, duration and which gauge view
to leave the board in afterwards. Everything else is buttons.

The board is put in its dual stream (x 3), so ALL 6 gauges are logged from
the same run:
  CH1  middle pair (2 SG),      POISSON_HALF, HX711 #1 on D2 / D3
  CH2  left + right pairs (4 SG), POISSON_FULL, HX711 #2 on D4 / D5
  combined = CH1 + g x CH2

Saved in ../data/strain_gauge/:
  <test>_<stamp>_dual.csv       host_time,t_ms,raw1,raw2,ustr1,ustr2,ustr_comb,marker
  <test>_<stamp>_summary.txt    noise, drift, slope and settling per signal
  <test>_<stamp>_plot.png       the three signals against time, with markers

Suggested tests
  drift      plate empty, 40 min from a cold start
  thermal    "Hand on left" / "Hand off" markers, 30 s each side
  zero ret.  "Load ON" / "Load OFF" markers, same mass 5 times
  creep      load on, 30 min
"""

import csv
import json
import re
import statistics
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "strain_gauge"
SETTINGS_FILE = HERE / "stm_dual_test_last.json"
BAUD = 115200
NAMES = ("CH1 middle 2 SG", "CH2 L+R 4 SG", "All 6 combined")
COLOURS = ("#1565c0", "#c62828", "#2e7d32")
RESTORE_CHOICES = {"All 6 combined (x 0)": "x 0", "Middle 2 SG (x 1)": "x 1",
                   "Left + right 4 SG (x 2)": "x 2"}
SETTLE_BAND = 0.5          # ustrain, for the warm-up settling time
MAX_PLOT_POINTS = 4000


# ======================================================================
#  start window
# ======================================================================
def load_last():
    try:
        return json.loads(SETTINGS_FILE.read_text())
    except Exception:
        return {}


def save_last(d):
    try:
        SETTINGS_FILE.write_text(json.dumps(d, indent=2))
    except Exception:
        pass


def next_test_name():
    n = 0
    if DATA_DIR.is_dir():
        for f in DATA_DIR.glob("test_*"):
            m = re.match(r"test_(\d+)", f.name)
            if m:
                n = max(n, int(m.group(1)))
    return f"test_{n + 1}"


def start_window():
    last = load_last()
    result = {}
    root = tk.Tk()
    root.title("STM32 dual bridge test - start")
    root.resizable(False, False)
    frm = ttk.Frame(root, padding=14)
    frm.pack(fill=tk.BOTH, expand=True)
    ttk.Label(frm, text="STM32 dual bridge drift test", font=("Segoe UI", 13, "bold")).grid(
        row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

    ttk.Label(frm, text="Serial port").grid(row=1, column=0, sticky="w", pady=4)
    port_var = tk.StringVar()
    port_box = ttk.Combobox(frm, textvariable=port_var, width=34)
    port_box.grid(row=1, column=1, sticky="we", pady=4)

    def refresh():
        ports = [(p.device, p.description) for p in list_ports.comports()]
        port_box["values"] = [f"{d}  -  {desc}" for d, desc in ports]
        want = last.get("port")
        pick = next((f"{d}  -  {desc}" for d, desc in ports if d == want), None)
        if pick is None and ports:
            pick = next((f"{d}  -  {desc}" for d, desc in ports
                         if re.search(r"stlink|st-link|stmicro|stm32", desc, re.I)), None) \
                   or f"{ports[0][0]}  -  {ports[0][1]}"
        port_var.set(pick or want or "")

    ttk.Button(frm, text="Refresh", width=9, command=refresh).grid(row=1, column=2, padx=(6, 0))
    refresh()

    ttk.Label(frm, text="Test name").grid(row=2, column=0, sticky="w", pady=4)
    test_var = tk.StringVar(value=next_test_name())
    ttk.Entry(frm, textvariable=test_var, width=36).grid(row=2, column=1, columnspan=2, sticky="we", pady=4)

    ttk.Label(frm, text="Duration").grid(row=3, column=0, sticky="w", pady=4)
    dur_var = tk.StringVar(value=last.get("duration", "40 min"))
    ttk.Combobox(frm, textvariable=dur_var, width=20,
                 values=["10 min", "20 min", "30 min", "40 min", "60 min", "until Stop"]).grid(
        row=3, column=1, sticky="w", pady=4)

    ttk.Label(frm, text="Afterwards show").grid(row=4, column=0, sticky="w", pady=4)
    after_var = tk.StringVar(value=last.get("after") if last.get("after") in RESTORE_CHOICES
                             else "All 6 combined (x 0)")
    ttk.Combobox(frm, textvariable=after_var, state="readonly", width=24,
                 values=list(RESTORE_CHOICES)).grid(row=4, column=1, sticky="w", pady=4)

    def start(_e=None):
        port = port_var.get().split("  -  ")[0].strip()
        test = test_var.get().strip()
        if not port:
            messagebox.showerror("Missing port", "Pick a serial port (plug the Nucleo in, then Refresh).")
            return
        if not test or re.search(r'[\\/:*?"<>|]', test):
            messagebox.showerror("Test name", "Enter a test name without \\ / : * ? \" < > |")
            return
        d = dur_var.get().strip()
        m = re.match(r"([\d.]+)", d)
        minutes = float(m.group(1)) if m else 0.0          # 0 = until Stop
        result.update(port=port, test=test, minutes=minutes, restore=RESTORE_CHOICES[after_var.get()])
        save_last({"port": port, "duration": d, "after": after_var.get()})
        root.destroy()

    b = ttk.Frame(frm)
    b.grid(row=5, column=0, columnspan=3, sticky="e", pady=(12, 0))
    ttk.Button(b, text="Cancel", command=root.destroy).pack(side=tk.RIGHT, padx=(6, 0))
    ttk.Button(b, text="Start", command=start).pack(side=tk.RIGHT)
    root.bind("<Return>", start)
    root.mainloop()
    return result or None


# ======================================================================
#  serial thread
# ======================================================================
class Board(threading.Thread):
    """Opens the port, puts the v4 sketch in dual-stream mode, collects rows."""

    def __init__(self, port, restore="x 0"):
        super().__init__(daemon=True)
        self.port = port
        self.restore = restore
        self.rows = deque()        # (host_time, t_ms, raw1, raw2, u1, u2, uc, pose, tilt)
        self.msgs = deque()        # '#' lines and notes for the log pane
        self.tx = deque()
        self.stop_flag = threading.Event()
        self.ok = threading.Event()        # first dual row seen
        self.error = None

    def send(self, cmd):
        self.tx.append(cmd)

    def run(self):
        try:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = self.port, BAUD, 0.2
            ser.dtr = False        # harmless on the ST-LINK port
            ser.rts = False
            ser.open()
        except serial.SerialException as e:
            self.error = f"Could not open {self.port}: {e}"
            self.msgs.append("# ERROR: " + self.error)
            return
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(b"\n")           # flush any junk the port open left in the board
        time.sleep(0.1)
        ser.write(b"LD 1\n")       # RotaTruck: enter the load-detect console
        time.sleep(0.3)
        ser.write(b"x 3\n")
        last_x = time.time()
        tries = 1
        buf = b""
        try:
            while not self.stop_flag.is_set():
                while self.tx:
                    cmd = self.tx.popleft()
                    ser.write((cmd + "\n").encode())
                    self.msgs.append(f">> sent: {cmd}")
                if not self.ok.is_set() and time.time() - last_x > 3.0:
                    if tries >= 6:
                        self.msgs.append("# No dual-stream data. Is firmware v4.1 flashed? "
                                         "Close the PlatformIO serial monitor and the dashboard.")
                        tries = 0
                    ser.write(b"\nLD 1\nx 3\n")
                    tries += 1
                    last_x = time.time()
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode(errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        self.msgs.append(line)
                        continue
                    if not line.startswith("D,"):
                        continue
                    p = line.split(",")
                    if len(p) not in (7, 9):          # 9 = firmware v4.2, with pose and tilt
                        continue
                    try:
                        pose, tilt = (int(p[7]), float(p[8])) if len(p) == 9 else (0, float("nan"))
                        row = (time.time(), int(p[1]), int(p[2]), int(p[3]),
                               float(p[4]), float(p[5]), float(p[6]), pose, tilt)
                    except ValueError:
                        continue
                    self.ok.set()
                    self.rows.append(row)
        finally:
            try:
                ser.write((self.restore + "\n").encode())
                time.sleep(0.3)
                ser.write(b"LD 0\n")   # RotaTruck: back to the motor console
                time.sleep(0.2)
                ser.close()
            except Exception:
                pass


# ======================================================================
#  analysis
# ======================================================================
def linear_slope(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0.0


def settle_time(t_min, ys, band):
    tail = [y for t, y in zip(t_min, ys) if t >= t_min[-1] - 5.0] or ys[-10:]
    final = statistics.fmean(tail)
    last_out = None
    for t, y in zip(t_min, ys):
        if abs(y - final) > band:
            last_out = t
    return 0.0 if last_out is None else last_out


def summarise(test, stamp, t_min, series, markers):
    lines = [f"Test: {test}   started {stamp}   duration {t_min[-1]:.1f} min   samples {len(t_min)}", ""]
    hdr = f"{'signal':<18}{'noise sd':>10}{'p-p':>10}{'drift':>10}{'slope/min':>11}{'last10 slope':>14}{'settled at':>12}"
    lines += [hdr, "-" * len(hdr)]
    win = 1.0 if t_min[-1] >= 3.0 else t_min[-1] / 5.0
    first = [i for i, t in enumerate(t_min) if t <= win] or [0]
    last = [i for i, t in enumerate(t_min) if t >= t_min[-1] - win] or [len(t_min) - 1]
    late = [i for i, t in enumerate(t_min) if t >= t_min[-1] - 10.0]
    for name, ys in zip(NAMES, series):
        diffs = [b - a for a, b in zip(ys, ys[1:])]
        noise = statistics.pstdev(diffs) / 2 ** 0.5 if len(diffs) > 1 else 0.0
        drift = statistics.fmean(ys[i] for i in last) - statistics.fmean(ys[i] for i in first)
        lines.append(f"{name:<18}{noise:>10.3f}{max(ys) - min(ys):>10.3f}{drift:>10.3f}"
                     f"{linear_slope(t_min, ys):>11.4f}"
                     f"{linear_slope([t_min[i] for i in late], [ys[i] for i in late]):>14.4f}"
                     f"{settle_time(t_min, ys, SETTLE_BAND):>10.1f} m")
    lines += ["", "All values in microstrain. noise sd = sample-to-sample noise.",
              "drift = mean of the last minute minus mean of the first minute.",
              f"settled at = time after which the signal stayed within +/-{SETTLE_BAND} ustrain "
              "of its final 5 min mean."]
    if markers:
        lines += ["", "Markers:"] + [f"  {t:7.2f} min  {txt}" for t, txt in markers]
    c1, c2 = series[0], series[1]
    if len(c1) > 20:
        d1 = abs(statistics.fmean(c1[-10:]) - statistics.fmean(c1[:10]))
        d2 = abs(statistics.fmean(c2[-10:]) - statistics.fmean(c2[:10]))
        lines += ["", "Verdict: " + ("CH2 drift is comparable to CH1." if d2 <= max(2 * d1, 1.0)
                                     else "CH2 drifts much more than CH1: check leads, solder joints and bonds.")]
    return "\n".join(lines)


# ======================================================================
#  main window
# ======================================================================
class DualTestApp:
    def __init__(self, port, test, minutes, restore="x 0"):
        self.test, self.minutes = test, minutes
        self.stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        base = DATA_DIR / f"{test}_{self.stamp}"
        self.csv_path = base.with_name(base.name + "_dual.csv")
        self.sum_path = base.with_name(base.name + "_summary.txt")
        self.png_path = base.with_name(base.name + "_plot.png")
        self.fh = open(self.csv_path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(["host_time", "t_ms", "raw1", "raw2", "ustr1", "ustr2", "ustr_comb", "marker",
                         "pose", "tilt_deg"])

        self.t0 = None
        self.t_min, self.s = [], ([], [], [])
        self.markers, self.pending = [], []
        self.recording = True
        self.finished = False
        self.last_draw = 0.0
        self.rate_n, self.rate_t, self.rate = 0, time.time(), 0.0

        self.board = Board(port, restore)
        self.board.start()

        self.root = tk.Tk()
        self.root.title(f"STM32 dual bridge test - {test}  ({port})")
        self.root.geometry("1360x860")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.build()
        self.root.after(200, self.tick)

    # ------------------------------------------------------------ layout
    def build(self):
        left = ttk.Frame(self.root, padding=6)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.fig.subplots_adjust(hspace=0.35, left=0.12, right=0.97, top=0.95, bottom=0.07)
        self.axes, self.lines = [], []
        for i, name in enumerate(NAMES):
            ax = self.fig.add_subplot(3, 1, i + 1)
            ax.set_ylabel(f"{name}\n(ustrain)", fontsize=9)
            ax.grid(alpha=0.3)
            (ln,) = ax.plot([], [], lw=1.0, color=COLOURS[i])
            self.axes.append(ax); self.lines.append(ln)
        self.axes[0].set_title("Unfiltered strain: middle 2 SG, left + right 4 SG, all 6", fontsize=10)
        self.axes[-1].set_xlabel("time (min)")
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        right = ttk.Frame(self.root, padding=10, width=440)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        self.banner = tk.Label(right, text="CONNECTING...", bg="#9e9e9e", fg="white",
                               font=("Segoe UI", 15, "bold"), height=2)
        self.banner.pack(fill=tk.X, pady=(0, 6))

        live = ttk.LabelFrame(right, text="Live", padding=8)
        live.pack(fill=tk.X, pady=(0, 6))
        self.live_var = tk.StringVar(value="waiting for data")
        tk.Label(live, textvariable=self.live_var, font=("Consolas", 10), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X)

        mk = ttk.LabelFrame(right, text="Markers", padding=6)
        mk.pack(fill=tk.X, pady=(0, 6))
        grid = ttk.Frame(mk)
        grid.pack(fill=tk.X)
        presets = ("Load ON", "Load OFF", "Hand on left", "Hand on right", "Hand off", "Disturbed")
        for i, txt in enumerate(presets):
            ttk.Button(grid, text=txt, command=lambda t=txt: self.add_marker(t)).grid(
                row=i // 3, column=i % 3, sticky="we", padx=2, pady=2)
        for c in range(3):
            grid.columnconfigure(c, weight=1)
        row = ttk.Frame(mk)
        row.pack(fill=tk.X, pady=(4, 0))
        self.note = ttk.Entry(row)
        self.note.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.note.bind("<Return>", lambda e: self.add_note())
        ttk.Button(row, text="Add note", command=self.add_note).pack(side=tk.LEFT, padx=(4, 0))

        bd = ttk.LabelFrame(right, text="Board", padding=6)
        bd.pack(fill=tk.X, pady=(0, 6))
        g2 = ttk.Frame(bd)
        g2.pack(fill=tk.X)
        actions = (("Diagnose both", "d", None),
                   ("Bridge report", "b", None),
                   ("Save zero (tipped back, empty)", "z", "zero saved"),
                   ("Flip CH1 sign", "i 1", "CH1 sign flipped"),
                   ("Flip CH2 sign", "i 2", "CH2 sign flipped"),
                   ("Show channels", "v", None),
                   ("Mass on MIDDLE strip", "g m", "g: mass on middle"),
                   ("Same mass on SIDE strip", "g s", "g: mass on side"))
        for i, (txt, cmd, mark) in enumerate(actions):
            ttk.Button(g2, text=txt, command=lambda c=cmd, m=mark: self.board_cmd(c, m)).grid(
                row=i // 2, column=i % 2, sticky="we", padx=2, pady=2)
        g2.columnconfigure(0, weight=1); g2.columnconfigure(1, weight=1)
        tk.Label(bd, text="Diagnose pauses the data for about 6 s.", font=("Segoe UI", 8),
                 fg="#555", anchor="w").pack(fill=tk.X)

        self.stop_btn = ttk.Button(right, text="STOP and save summary", command=self.finish)
        self.stop_btn.pack(fill=tk.X, pady=(0, 6))

        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)
        t1 = ttk.Frame(nb); nb.add(t1, text="Board messages")
        self.log = tk.Text(t1, height=10, font=("Consolas", 8), wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)
        t2 = ttk.Frame(nb); nb.add(t2, text="Summary")
        self.sum_text = tk.Text(t2, height=10, font=("Consolas", 8), wrap=tk.NONE)
        self.sum_text.pack(fill=tk.BOTH, expand=True)
        self.nb = nb

        self.status = tk.StringVar(value=f"logging to {self.csv_path.name}")
        tk.Label(right, textvariable=self.status, font=("Consolas", 8), anchor="w",
                 fg="#555").pack(fill=tk.X, pady=(4, 0))

    # ------------------------------------------------------------ actions
    def add_marker(self, text):
        if self.recording:
            self.pending.append(text)

    def add_note(self):
        t = self.note.get().strip()
        if t:
            self.add_marker(t)
            self.note.delete(0, tk.END)

    def board_cmd(self, cmd, mark):
        if self.finished:
            return
        if cmd == "z" and not messagebox.askyesno("Save zero", "Plate EMPTY and trolley resting tipped back, hands off?\n"
                                                  "This saves the zero for both bridges."):
            return
        if cmd in ("i 1", "i 2") and not messagebox.askyesno(
                "Flip sign", f"Flip the sign of {'CH1' if cmd == 'i 1' else 'CH2'}?\n"
                "Do this only if loading that bridge makes it go DOWN."):
            return
        self.board.send(cmd)
        if mark:
            self.add_marker(mark)

    # ------------------------------------------------------------ loop
    def tick(self):
        while self.board.msgs:
            self.log.insert(tk.END, self.board.msgs.popleft() + "\n")
            self.log.see(tk.END)
            if float(self.log.index("end-1c").split(".")[0]) > 600:
                self.log.delete("1.0", "200.0")

        new = 0
        while self.board.rows:
            ht, t_ms, r1, r2, u1, u2, uc, pose, tilt = self.board.rows.popleft()
            if not self.recording:
                continue
            if self.t0 is None:
                self.t0 = ht
            tm = (ht - self.t0) / 60.0
            mark = ""
            if self.pending:
                mark = self.pending.pop(0)
                self.markers.append((tm, mark))
                for ax in self.axes:
                    ax.axvline(tm, color="gray", ls="--", lw=0.8)
                self.axes[0].text(tm, 1.0, " " + mark, rotation=90, va="top", fontsize=7,
                                  transform=self.axes[0].get_xaxis_transform())
            self.w.writerow([f"{ht:.3f}", t_ms, r1, r2, u1, u2, uc, mark, pose,
                             "" if tilt != tilt else f"{tilt:.1f}"])
            self.t_min.append(tm)
            for lst, v in zip(self.s, (u1, u2, uc)):
                lst.append(v)
            new += 1
        if new:
            self.fh.flush()
            self.rate_n += new

        now = time.time()
        if now - self.rate_t >= 2.0:
            self.rate = self.rate_n / (now - self.rate_t)
            self.rate_n, self.rate_t = 0, now

        self.update_banner(now)
        if self.t_min and self.recording:
            self.update_live()
            if now - self.last_draw > 1.0:
                self.redraw()
                self.last_draw = now
            if self.minutes > 0 and self.t_min[-1] >= self.minutes:
                self.finish()

        self.root.after(200, self.tick)

    def update_banner(self, now):
        if self.finished:
            txt, col = "STOPPED - SUMMARY SAVED", "#455a64"
        elif self.board.error:
            txt, col = "PORT ERROR", "#c62828"
        elif not self.board.ok.is_set():
            txt, col = "WAITING FOR BOARD (v4.1)...", "#9e9e9e"
        elif self.rate < 2.0 and self.t_min:
            txt, col = "NO DATA / PAUSED", "#ef6c00"
        else:
            txt, col = "RECORDING", "#2e7d32"
        self.banner.config(text=txt, bg=col)

    def update_live(self):
        tm = self.t_min[-1]
        left = f"{self.minutes - tm:5.1f} min left" if self.minutes > 0 else "until Stop"
        rows = [f"elapsed {tm:6.1f} min   {left}   {self.rate:4.1f} Hz", ""]
        rows.append(f"{'':16}{'now':>10}{'drift':>10}")
        for name, lst in zip(NAMES, self.s):
            rows.append(f"{name:<16}{lst[-1]:>10.2f}{lst[-1] - lst[0]:>+10.2f}")
        rows += ["", "ustrain, unfiltered. drift = since the start of the test."]
        self.live_var.set("\n".join(rows))

    def redraw(self):
        n = len(self.t_min)
        step = max(1, n // MAX_PLOT_POINTS)
        t = self.t_min[::step]
        for ax, ln, lst in zip(self.axes, self.lines, self.s):
            ln.set_data(t, lst[::step])
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    # ------------------------------------------------------------ finish
    def finish(self):
        if self.finished:
            return
        self.recording = False
        self.finished = True
        self.board.stop_flag.set()            # also returns the board to the chosen view
        self.stop_btn.config(state=tk.DISABLED)
        self.fh.close()
        if len(self.t_min) < 10:
            text = ("Not enough samples for a summary.\n"
                    "Check the Nucleo runs firmware v4.1 and both HX711s respond (Diagnose both).")
        else:
            self.redraw()
            text = summarise(self.test, self.stamp, self.t_min, list(self.s), self.markers)
            self.sum_path.write_text(text)
            self.fig.savefig(self.png_path, dpi=150)
            text += f"\n\nSaved:\n  {self.csv_path.name}\n  {self.sum_path.name}\n  {self.png_path.name}"
        self.sum_text.delete("1.0", tk.END)
        self.sum_text.insert(tk.END, text)
        self.nb.select(1)
        self.status.set(f"saved in {DATA_DIR}")

    def on_close(self):
        if not self.finished and self.t_min:
            if not messagebox.askyesno("Stop test", "Stop the test and save the summary?"):
                return
        self.finish()
        self.root.after(400, self.root.destroy)

    def run(self):
        self.root.mainloop()


def main():
    choice = start_window()
    if choice:
        DualTestApp(choice["port"], choice["test"], choice["minutes"], choice["restore"]).run()


if __name__ == "__main__":
    main()

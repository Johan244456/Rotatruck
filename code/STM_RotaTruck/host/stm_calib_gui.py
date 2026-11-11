"""
Calibration dashboard for the P102 rig on the NUCLEO-G491RE (firmware v4.1).
STM copy of calib_gui.py, with:
  - a GAUGES switch: middle pair (2 SG), left + right pairs (4 SG), or all 6
    combined. Each sends 'x <n>' to the board; each view keeps its own scale,
    so the three never overwrite each other. RotaTruck: the board KEEPS its
    saved scale + offset across launches (the assist uses it). Type 'u' in the
    command box to clear every view on purpose.
  - two-bridge buttons: show both bridges, flip a channel's sign, balance g
  - auto-zero on / off (off while you watch drift by eye)

Built on Tkinter with matplotlib embedded, rather than matplotlib's own
widgets. The matplotlib TextBox widget throws AttributeError on window
resize (ResizeEvent has no .inaxes), which made it unusable here.

Provides:
  - live readout + rolling trace
  - sensor health classifier (detected / railed / frozen / no data)
  - live reading log and board message log
  - tare
  - single-shot capture, and a DRIFT-SAFE paired capture
  - multi-point calibration with least-squares fit and R^2
  - residual plot so nonlinearity is visible, not just assumed
  - a command box, so d / b / n / f / k / m / h / r can be sent without the
    Arduino monitor, plus buttons for the stable-hold / smooth filter modes
  - completion resistor buttons (e.g. 330 / 350 ohm), shown when the
    config supplies "comp_choices"
  - a guard that compares the host's counts/ustrain and completion
    resistor against the board's

v3 notes
  PAIRED CAPTURE is the one to use. A single-shot capture is referenced to
  a tare taken minutes earlier, so any drift between the tare and the
  capture lands in the calibration slope and inflates the sensitivity.
  Paired capture records unloaded, then loaded, and stores the difference,
  so only the seconds between the two carry drift.

  The board does not reset when this connects (dtr/rts are held low), so
  the boot banner has usually gone past. bench_common sends 'b' on connect
  to make the board reprint it.
"""

import csv
import json
import statistics
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
import tkinter.messagebox
from collections import deque
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from stm_bench_common import connection_status, linear_fit

AVERAGE_SAMPLES = 40          # samples averaged for the LOADED half of a point
ZERO_SAMPLES    = 25          # samples averaged for the UNLOADED half
HISTORY_SECONDS = 30
UPDATE_MS = 100
HEALTH_WINDOW = 40            # samples used to judge sensor health
HX711_RAIL = 8_300_000        # true rail is 8,388,607; warn before it
COMPLETION_CONFIGS = ("QUARTER", "POISSON_HALF", "BENDING_HALF")
# pose column from firmware v4.2 (0 = no IMU or older firmware)
POSE_NAMES = {0: "unknown", 1: "STANDING", 2: "TILTING", 3: "TIPPED BACK (still)",
              4: "TIPPED BACK (moving)"}
POSE_STANDING, POSE_TIPPED_STILL = 1, 3


class CalibrationDashboard:
    def __init__(self, reader, config):
        self.reader = reader
        self.cfg = config
        # Known loads are TYPED in input_unit (e.g. grams) and converted to
        # the fit unit (e.g. newtons) with input_to_unit, so the slope sent
        # to the board stays counts per newton.
        self.in_unit = config.get("input_unit", config["unit"])
        self.in_to_unit = config.get("input_to_unit", 1.0)
        self.points = []           # (known in input_unit, tared counts, method)
        self.capture_buffer = None
        self.capture_target = None
        self.raw_buffer = deque(maxlen=HEALTH_WINDOW)
        self.fitted_scale = None
        self._last_t = None

        # paired capture state: None | "zero" | "await_load" | "load"
        self.pair_state = None
        self.pair_zero = None

        self._mismatch_checked = False

        hist_len = int(HISTORY_SECONDS * 1000 / UPDATE_MS)
        self.t_hist = deque(maxlen=hist_len)
        self.v_hist = deque(maxlen=hist_len)

        self.root = tk.Tk()
        self.root.title(config["title"])
        self.root.geometry("1720x960")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_left()
        self._build_right()
        self.calibrated = False
        self.session_scales = {}       # view key -> (counts/N, offset) applied in THIS session only
        self._reset_calibration()
        self._load_final()
        self._tick()

    # ------------------------------------------------------------ calibration storage
    def _calib_path(self):
        p = self.cfg.get("calib_file")
        return Path(p) if p else None

    def _reset_calibration(self):
        """On launch: start a fresh set of capture points, but KEEP the board's
        saved scale + offset (RotaTruck: the assist relies on it, and opening
        the dashboard just to look must not uncalibrate the truck). The board
        banner shows what is saved. Send 'u' from the command box to clear
        every view on purpose. The JSON files are written on APPLY as a record."""
        self.fitted_scale = None
        self.fitted_offset = 0.0
        self.calibrated = False
        self.fit_var.set("Board keeps its saved calibration (see 'saved scale' in Board messages).\n"
                         "To recalibrate: capture at least 3 paired points, then APPLY.")
        self.capture_status.set("Saved board calibration kept. Calibrate + APPLY to replace it "
                                "('u' in the command box clears all views).")

    # ------------------------------------------------------------ FINAL calibration
    def _load_final(self):
        """Launch / view switch: load this view's accepted calibration
        (*_FINAL.json) - its points and fit go into the plot and table, and
        _sync_final() makes sure the board's flash holds the same scale + offset.
        Capturing more points extends it; 'Clear all' starts fresh."""
        self.final = None
        self.final_state = "idle"
        self._final_t = time.time()
        path = self.cfg.get("final_file")
        if not path or not Path(path).exists():
            return
        try:
            d = json.loads(Path(path).read_text())
            m, c = float(d["counts_per_newton"]), float(d["offset_counts"])
            pts = [(float(p["known_g"]), float(p["counts"]), p.get("method", "single"))
                   for p in d.get("points", [])]
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.capture_status.set(f"Could not read {Path(path).name}: {e}")
            return
        self.final = {"m": m, "c": c, "file": Path(path).name, "saved": d.get("saved", "?"),
                      "gain": d.get("hx711_gain")}
        self.final_state = "wait_board"
        self.points = sorted(pts, key=lambda p: p[0])
        self._refresh_tree()
        self._recompute_fit()
        self.fitted_scale, self.fitted_offset = m, c      # exactly the accepted values
        self.calibrated = True
        self.session_scales[self.cfg.get("view_key")] = (m, c)
        self.capture_status.set(
            f"FINAL calibration loaded from {self.final['file']} (saved {self.final['saved']}): "
            f"{m:.4f} counts/N, offset {c:.0f} counts. Checking the board has it...")

    def _sync_final(self):
        """Once the board reports the saved scale of this view: if it differs from
        the FINAL file (cleared with 'u', or a different APPLY), send it back."""
        if not getattr(self, "final", None) or self.final_state in ("idle", "done"):
            return
        view = int(self.cfg.get("view_key", 0))
        if self.reader.board_fact("view") != view:
            return                                        # 'x <view>' not confirmed yet
        if self.final_state == "wait_board":
            if self.reader.board_fact("saved_scale_view") != view:
                self.reader.send("s")                     # no args: print the saved calibration
                self.final_state = "queried"
                self._final_t = time.time()
                return
            self.final_state = "queried"
            self._final_t = time.time()
        if self.reader.board_fact("saved_scale_view") != view:
            if time.time() - self._final_t > 5.0:         # ask again if the reply got lost
                self.final_state = "wait_board"
            return
        m, c = self.final["m"], self.final["c"]
        bs, bo = self.reader.board_fact("saved_scale"), self.reader.board_fact("saved_offset")
        if bs is not None and bo is None and time.time() - self._final_t < 3.0:
            return                                        # offset line not in yet
        gain = round(128 * self._gain_ratio())
        if self.final.get("gain") and self.reader.board_fact("hx_fs_mv") and gain != self.final["gain"]:
            self.capture_status.set(
                f"FINAL calibration NOT applied: it was made at HX711 gain {self.final['gain']}, "
                f"the board runs gain {gain}. Recalibrate.")
            self.final_state = "done"
            return
        if bs is not None and abs(bs - m) <= 1e-3 * abs(m) and bo is not None and abs(bo - c) <= 1.0:
            self.capture_status.set(
                f"FINAL calibration in use: board flash already holds {m:.4f} counts/N, offset "
                f"{c:.0f} counts ({self.final['file']}).")
        else:
            self.reader.send(f"{self.cfg['scale_command']} {m:.4f} {c:.1f}")
            self.reader.forget_fact("saved_scale_view")
            self.capture_status.set(
                f"Board had {'no scale' if bs is None else f'{bs:.4f} counts/N, offset {bo}'}: "
                f"restored the FINAL calibration ({m:.4f} counts/N, offset {c:.0f}) from "
                f"{self.final['file']} to flash.")
        self.final_state = "done"

    def _restore_session_calibration(self):
        """After a view switch, show this session's scale for the new view, if
        any. The board already holds it in flash from the APPLY, so nothing is sent."""
        saved = self.session_scales.get(self.cfg.get("view_key"))
        if saved is None:
            return
        cpn, off = saved
        self.fitted_scale, self.fitted_offset = cpn, off
        self.calibrated = True
        self.fit_var.set(f"Calibrated earlier this session:\n"
                         f"counts per {self.cfg['unit']} = {cpn:,.3f}\n"
                         f"offset = {off:,.0f} counts ({self._offset_kg(cpn, off):+.2f} kg at the zero)")

    def _offset_kg(self, cpn, off):
        """Load the board reports at the zero (tipped back, empty): -offset / scale."""
        return -off / cpn / 9.80665 if cpn else 0.0

    def _pose_now(self):
        """Pose code from the board (0 = no IMU / older firmware)."""
        p = self.reader.snapshot().get("pose", 0)
        try:
            return int(p) if p == p else 0
        except (TypeError, ValueError):
            return 0

    def _pose_ok(self, expect, what):
        """Refuse a capture taken in the wrong pose. No IMU data = no check."""
        if self.reader.board_fact("pose_gating") is False:      # no-IMU mode: you place it
            return True
        p = self._pose_now()
        if p == 0 or p == expect:
            return True
        self.capture_status.set(
            f"Not captured: the trolley is {POSE_NAMES.get(p, p)}. {what}")
        return False

    def _save_calibration(self, cpn, extra=None):
        path = self._calib_path()
        if not path:
            return
        data = {
            "counts_per_newton": cpn,
            "saved": datetime.now().isoformat(timespec="seconds"),
            "source": Path(str(getattr(self.reader, "filename", "dashboard"))).name,
            "unit": self.cfg["unit"],
            "input_unit": self.in_unit,
            "points": [{"known": k, "counts": c, "method": m} for k, c, m in self.points],
        }
        if extra:
            data.update(extra)
        path.write_text(json.dumps(data, indent=2))

    # ------------------------------------------------------------ layout
    def _build_left(self):
        left = ttk.Frame(self.root, padding=6)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(7.0, 7.6), dpi=100)
        self.fig.subplots_adjust(hspace=0.5, left=0.15, right=0.96,
                                 top=0.95, bottom=0.07)

        self.ax_live = self.fig.add_subplot(3, 1, 1)
        self.ax_live.set_title("Live reading", fontsize=10)
        self.ax_live.set_ylabel(self.cfg["unit"])
        self.ax_live.axhline(0, color="gray", lw=0.8, ls=":")
        (self.live_line,) = self.ax_live.plot([], [], lw=1.4, color="#1565c0")

        self.ax_fit = self.fig.add_subplot(3, 1, 2)
        self.ax_fit.set_title("Calibration points and fit", fontsize=10)
        self.ax_fit.set_xlabel(f"Known load ({self.in_unit})")
        self.ax_fit.set_ylabel("Tared counts")
        (self.fit_pts,) = self.ax_fit.plot([], [], "o", color="#6a1b9a", ms=7)
        (self.fit_line,) = self.ax_fit.plot([], [], "-", color="#c62828", lw=1.3)

        self.ax_res = self.fig.add_subplot(3, 1, 3)
        self.ax_res.set_title("Fit residuals  (curvature here = nonlinearity)", fontsize=10)
        self.ax_res.set_xlabel(f"Known load ({self.in_unit})")
        self.ax_res.set_ylabel("counts")
        self.ax_res.axhline(0, color="gray", lw=0.8, ls=":")
        (self.res_pts,) = self.ax_res.plot([], [], "o", color="#ef6c00", ms=6)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_right(self):
        # Scrollable right panel, so Step 3 / APPLY is reachable on short screens.
        outer = ttk.Frame(self.root, width=760)
        outer.pack(side=tk.RIGHT, fill=tk.Y)
        outer.pack_propagate(False)
        canvas = tk.Canvas(outer, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(canvas, padding=10)
        win = canvas.create_window((0, 0), window=right, anchor="nw")
        right.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(win, width=e.width))

        def _on_wheel(event):
            w = self.root.winfo_containing(event.x_root, event.y_root)
            if w is None or isinstance(w, tk.Text):
                return                      # text logs scroll themselves
            if not (str(w) == str(outer) or str(w).startswith(str(outer) + ".")):
                return                      # pointer is over the plots
            if getattr(event, "num", None) == 4:
                step = -1
            elif getattr(event, "num", None) == 5:
                step = 1
            else:
                step = -1 if event.delta > 0 else 1
            canvas.yview_scroll(step * 3, "units")

        self.root.bind_all("<MouseWheel>", _on_wheel)     # Windows / macOS
        self.root.bind_all("<Button-4>", _on_wheel)       # Linux
        self.root.bind_all("<Button-5>", _on_wheel)

        # ---- sensor health banner ----
        self.health_frame = tk.Frame(right, bg="#9e9e9e", height=54)
        self.health_frame.pack(fill=tk.X, pady=(0, 6))
        self.health_frame.pack_propagate(False)
        self.health_var = tk.StringVar(value="WAITING FOR DATA")
        self.health_lbl = tk.Label(self.health_frame, textvariable=self.health_var,
                                   bg="#9e9e9e", fg="white",
                                   font=("Segoe UI", 15, "bold"))
        self.health_lbl.pack(expand=True)
        self.health_detail = tk.StringVar(value="")
        tk.Label(right, textvariable=self.health_detail, font=("Consolas", 8),
                 fg="#555", justify=tk.LEFT, anchor="w").pack(fill=tk.X, pady=(0, 2))

        # ---- host vs board agreement ----
        self.match_var = tk.StringVar(value="board banner not seen yet")
        self.match_lbl = tk.Label(right, textvariable=self.match_var,
                                  font=("Consolas", 8), fg="#777",
                                  justify=tk.LEFT, anchor="w", wraplength=720)
        self.match_lbl.pack(fill=tk.X, pady=(0, 6))

        # ---- gauge view switch (2 / 4 / 6 strain gauges) ----
        views = self.cfg.get("views", {})
        if views:
            vbox = ttk.LabelFrame(right, text="Gauges shown", padding=6)
            vbox.pack(fill=tk.X, pady=(0, 8))
            self.view_btns = {}
            vrow = ttk.Frame(vbox)
            vrow.pack(fill=tk.X)
            for key, v in views.items():
                b = tk.Button(vrow, text=v["button"], relief=tk.RAISED,
                              command=lambda k=key: self._switch_view(k))
                b.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
                self.view_btns[key] = b
            self.view_info = tk.StringVar(value="")
            tk.Label(vbox, textvariable=self.view_info, font=("Segoe UI", 8), fg="#555",
                     justify=tk.LEFT, anchor="w", wraplength=720).pack(fill=tk.X, pady=(4, 0))
            self._mark_view()

        # ---- big readout ----
        box = ttk.LabelFrame(right, text="Live", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        self.big_var = tk.StringVar(value="--")
        self.big_lbl = tk.Label(box, textvariable=self.big_var,
                                font=("Segoe UI", 34, "bold"))
        self.big_lbl.pack()
        self.big_sub_var = tk.StringVar(value="")
        self.big_sub_lbl = tk.Label(box, textvariable=self.big_sub_var,
                                    font=("Segoe UI", 12), fg="#555")
        self.big_sub_lbl.pack(pady=(0, 4))
        self.detail_var = tk.StringVar(value="")
        tk.Label(box, textvariable=self.detail_var, font=("Consolas", 9),
                 justify=tk.LEFT, anchor="w").pack(fill=tk.X)

        # ---- payload validity and assist (sketch v3.6) ----
        self.assist_on = bool(self.cfg.get("assist"))
        if self.assist_on:
            abox = ttk.LabelFrame(right, text="Assist output (safety factor from board, LOAD_ASSIST_SCALE)",
                                  padding=8)
            abox.pack(fill=tk.X, pady=(0, 8))
            self.assist_state_var = tk.StringVar(value="payload: waiting for board")
            self.assist_state_lbl = tk.Label(abox, textvariable=self.assist_state_var,
                                             font=("Segoe UI", 12, "bold"))
            self.assist_state_lbl.pack(anchor="w")
            self.assist_big_var = tk.StringVar(value="--")
            self.assist_big_lbl = tk.Label(abox, textvariable=self.assist_big_var,
                                           font=("Segoe UI", 20, "bold"))
            self.assist_big_lbl.pack(anchor="w")
            # load vs rated payload visualiser
            self.load_pct_var = tk.StringVar(value="load: --")
            tk.Label(abox, textvariable=self.load_pct_var,
                     font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(4, 0))
            self.load_canvas = tk.Canvas(abox, height=74, bg="white",
                                         highlightthickness=0)
            self.load_canvas.pack(fill=tk.X, pady=(2, 4))
            self.assist_sub_var = tk.StringVar(value="")
            tk.Label(abox, textvariable=self.assist_sub_var, font=("Consolas", 9),
                     justify=tk.LEFT, anchor="w").pack(fill=tk.X)

        # ---- board commands ----
        cmd = ttk.LabelFrame(right, text="Board commands", padding=6)
        cmd.pack(fill=tk.X, pady=(0, 6))
        qrow = ttk.Frame(cmd)
        qrow.pack(fill=tk.X)
        for label, c in (("diagnose (d)", "d"), ("bridge (b)", "b"),
                         ("drift 10 min (n 600)", "n 600")):
            ttk.Button(qrow, text=label, width=17,
                       command=lambda cc=c: self._send(cc)).pack(side=tk.LEFT, padx=2)

        # completion resistor switch
        self.comp_var = tk.StringVar(value="board: ?")
        choices = self.cfg.get("comp_choices", ())
        if choices:
            rrow = ttk.Frame(cmd)
            rrow.pack(fill=tk.X, pady=(4, 0))
            ttk.Label(rrow, text="Completion R:").pack(side=tk.LEFT)
            for v in choices:
                ttk.Button(rrow, text=f"{v:g} ohm", width=9,
                           command=lambda vv=v: self._set_comp(vv)).pack(side=tk.LEFT, padx=2)
            ttk.Label(rrow, textvariable=self.comp_var,
                      font=("Consolas", 9)).pack(side=tk.LEFT, padx=8)

        frow = ttk.Frame(cmd)
        frow.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(frow, text="Filter:").pack(side=tk.LEFT)
        for label, c in (("stable hold (m 1)", "m 1"), ("smooth (m 0)", "m 0")):
            ttk.Button(frow, text=label, width=17,
                       command=lambda cc=c: self._send(cc)).pack(side=tk.LEFT, padx=2)

        brow = ttk.Frame(cmd)
        brow.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(brow, text="Bridges:").pack(side=tk.LEFT)
        for label, c in (("show both (v)", "v"), ("flip CH1 sign", "i 1"),
                         ("flip CH2 sign", "i 2")):
            ttk.Button(brow, text=label, width=14,
                       command=lambda cc=c: self._two_bridge_cmd(cc)).pack(side=tk.LEFT, padx=2)
        grow = ttk.Frame(cmd)
        grow.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(grow, text="Balance g:").pack(side=tk.LEFT)
        ttk.Button(grow, text="1. mass on MIDDLE strip", width=22,
                   command=lambda: self._two_bridge_cmd("g m")).pack(side=tk.LEFT, padx=2)
        ttk.Button(grow, text="2. same mass on SIDE", width=20,
                   command=lambda: self._two_bridge_cmd("g s")).pack(side=tk.LEFT, padx=2)
        arow = ttk.Frame(cmd)
        arow.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(arow, text="Zero tracking:").pack(side=tk.LEFT)
        ttk.Button(arow, text="on (+/-3 kg)", width=17,
                   command=lambda: self._two_bridge_cmd("a 3")).pack(side=tk.LEFT, padx=2)
        ttk.Button(arow, text="off (watch drift)", width=17,
                   command=lambda: self._two_bridge_cmd("a 0")).pack(side=tk.LEFT, padx=2)
        irow = ttk.Frame(cmd)
        irow.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(irow, text="IMU mode:").pack(side=tk.LEFT)
        ttk.Button(irow, text="no IMU (assume standing)", width=24,
                   command=lambda: self._set_imu_mode(False)).pack(side=tk.LEFT, padx=2)
        ttk.Button(irow, text="use IMU", width=10,
                   command=lambda: self._set_imu_mode(True)).pack(side=tk.LEFT, padx=2)
        self.imu_mode_var = tk.StringVar(value="mode: waiting for board")
        self.imu_mode_lbl = tk.Label(irow, textvariable=self.imu_mode_var, font=("Segoe UI", 9, "bold"))
        self.imu_mode_lbl.pack(side=tk.LEFT, padx=6)

        # ---- simulated pose (env:loadcell_test only, bench tests without an IMU) ----
        srow = ttk.Frame(cmd)
        srow.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(srow, text="Simulated pose (no IMU):").pack(side=tk.LEFT)
        for label, m in (("Standing", "s"), ("Tipped back", "r"), ("Tilting", "t"),
                         ("off (real IMU)", "0")):
            ttk.Button(srow, text=label, width=13 if m != "0" else 14,
                       command=lambda m=m: self._sim_pose(m)).pack(side=tk.LEFT, padx=2)
        tk.Label(cmd, text="Bench only (loadcell_test firmware): stands in for the IMU so the "
                           "tipped-back zero / standing capture logic runs as on the truck. "
                           "Keep IMU mode on 'use IMU' (gating on).",
                 font=("Segoe UI", 8), fg="#777", justify=tk.LEFT, anchor="w",
                 wraplength=710).pack(fill=tk.X)

        crow = ttk.Frame(cmd)
        crow.pack(fill=tk.X, pady=(4, 0))
        self.cmd_entry = ttk.Entry(crow)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self._send_entry())
        ttk.Button(crow, text="Send", command=self._send_entry).pack(side=tk.LEFT, padx=4)

        # ---- tare ----
        tare_box = ttk.LabelFrame(right, text="Step 1 - zero", padding=6)
        tare_box.pack(fill=tk.X, pady=(0, 6))
        trow = ttk.Frame(tare_box)
        trow.pack(fill=tk.X)
        ttk.Button(trow, text="TARE (temporary)",
                   command=self._do_tare).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(trow, text="SAVE ZERO (tipped back, empty)",
                   command=self._save_zero).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))
        tk.Label(tare_box, text="SAVE ZERO is permanent (board flash, used at every power-up).\n"
                 "Do it with the plate empty and the trolley resting tipped back, hands off.\n"
                 "With the IMU, the zero then follows by itself whenever it rests tipped back.",
                 font=("Segoe UI", 8), fg="#555", justify=tk.LEFT, anchor="w").pack(fill=tk.X)

        # ---- capture points ----
        cap = ttk.LabelFrame(right, text="Step 2 - capture calibration points", padding=6)
        cap.pack(fill=tk.X, pady=(0, 6))
        row = ttk.Frame(cap)
        row.pack(fill=tk.X, pady=3)
        ttk.Label(row, text=f"Known ({self.in_unit}):").pack(side=tk.LEFT)
        self.known_entry = ttk.Entry(row, width=10)
        self.known_entry.pack(side=tk.LEFT, padx=6)
        self.known_entry.bind("<Return>", lambda e: self._start_pair())
        self.pair_btn = ttk.Button(row, text="Capture zero + load",
                                   command=self._start_pair)
        self.pair_btn.pack(side=tk.LEFT)
        ttk.Button(row, text="single", width=7,
                   command=self._capture_point).pack(side=tk.LEFT, padx=4)

        self.capture_status = tk.StringVar(
            value="Paired capture is drift safe. Use 'single' only for a quick look.")
        tk.Label(cap, textvariable=self.capture_status, fg="#1565c0",
                 font=("Segoe UI", 9), wraplength=710,
                 justify=tk.LEFT, anchor="w").pack(fill=tk.X)

        self.tree = ttk.Treeview(cap, columns=("known", "counts", "how"),
                                 show="headings", height=5)
        self.tree.heading("known", text=f"Known ({self.in_unit})")
        self.tree.heading("counts", text="Tared counts")
        self.tree.heading("how", text="method")
        self.tree.column("known", width=110, anchor="e")
        self.tree.column("counts", width=150, anchor="e")
        self.tree.column("how", width=70, anchor="center")
        self.tree.pack(fill=tk.X, pady=3)

        btns = ttk.Frame(cap)
        btns.pack(fill=tk.X)
        ttk.Button(btns, text="Delete selected",
                   command=self._delete_point).pack(side=tk.LEFT)
        ttk.Button(btns, text="Clear all",
                   command=self._clear_points).pack(side=tk.LEFT, padx=6)
        self.apply_btn_quick = ttk.Button(btns, text="APPLY scale to board",
                                          command=self._apply_scale, state=tk.DISABLED)
        self.apply_btn_quick.pack(side=tk.RIGHT)

        # ---- fit result ----
        fitbox = ttk.LabelFrame(right, text="Step 3 - fit and apply", padding=6)
        fitbox.pack(fill=tk.X, pady=(0, 6))
        self.fit_var = tk.StringVar(value="Capture at least 2 points.")
        tk.Label(fitbox, textvariable=self.fit_var, font=("Consolas", 9),
                 justify=tk.LEFT, anchor="w", wraplength=710).pack(fill=tk.X)
        self.apply_btn = ttk.Button(fitbox, text="APPLY scale to board",
                                    command=self._apply_scale, state=tk.DISABLED)
        self.apply_btn.pack(fill=tk.X, pady=3)

        # ---- logs, tabbed to save vertical space ----
        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        live_tab = ttk.Frame(nb)
        nb.add(live_tab, text="Live readings")
        self.live_log = tk.Text(live_tab, height=8, font=("Consolas", 8), wrap=tk.NONE)
        self.live_log.pack(fill=tk.BOTH, expand=True)

        msg_tab = ttk.Frame(nb)
        nb.add(msg_tab, text="Board messages")
        self.log = tk.Text(msg_tab, height=8, font=("Consolas", 8), wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

        self.status_var = tk.StringVar(value="")
        self.status_lbl = tk.Label(right, textvariable=self.status_var,
                                   font=("Consolas", 9), anchor="w")
        self.status_lbl.pack(fill=tk.X, pady=(4, 0))

    # ------------------------------------------------------------ health
    def _assess_health(self, snap, conn_label):
        """Returns (text, colour, detail) describing the sensor channel."""
        if conn_label != "LIVE":
            return "NO DATA FROM BOARD", "#616161", "Serial stalled or board not running."

        if len(self.raw_buffer) < 5:
            return "STARTING UP...", "#9e9e9e", "Collecting samples."

        raws = list(self.raw_buffer)
        latest = raws[-1]
        spread = max(raws) - min(raws)
        sd = statistics.pstdev(raws) if len(raws) > 1 else 0.0

        if abs(latest) >= HX711_RAIL:
            return ("SENSOR RAILED", "#c62828",
                    f"raw pinned at {latest:,.0f}. The bridge offset exceeds the\n"
                    f"HX711 ceiling of {self.cfg.get('fs_mv', 16.8):.1f} mV. Check the completion pair\n"
                    f"(CH1) or an open arm. Click diagnose (d) and bridge (b).")

        if spread == 0:
            return ("SIGNAL FROZEN", "#c62828",
                    "Raw value identical across every sample. HX711 is likely\n"
                    "not converting: check DT / SCK (CH1 PE11/PE12, CH2 PE13/PE14), 3V, GND.")

        detail = (f"noise sd = {sd:,.0f} counts  ({sd / self._cpu():.3f} "
                  f"{self.cfg['unit_small']})\n"
                  f"spread over last {len(raws)} samples = {spread:,.0f} counts")
        return "SENSOR DETECTED", "#2e7d32", detail

    def _gain_ratio(self):
        """HX711 gain on the board / 128, from its banner (AVDD + full scale).
        The view's counts/ustrain are computed for gain 128."""
        avdd = self.reader.board_fact("hx_avdd")
        fs = self.reader.board_fact("hx_fs_mv")
        if not avdd or not fs:
            return 1.0
        gain = round(0.5 * avdd * 1000.0 / fs)
        return gain / 128.0 if gain in (32, 64, 128) else 1.0

    def _cpu(self):
        """counts/ustrain for the current view at the board's gain."""
        return self.cfg["counts_per_unit"] * self._gain_ratio()

    def _check_board_match(self):
        """Warn loudly if the host's config or completion disagrees with the sketch's."""
        r2 = self.reader.board_fact("comp_r2")
        r4 = self.reader.board_fact("comp_r4")
        if r2 is not None and r4 is not None:
            self.comp_var.set(f"board: {r2:g} / {r4:g} ohm")

        board_cpu = self.reader.board_fact("counts_per_ustrain")
        if board_cpu is None:
            return
        host_cpu = self._cpu()
        fs = self.reader.board_fact("hx_fs_mv")
        if fs:
            self.cfg["fs_mv"] = fs
        cfg_name = self.reader.board_fact("config", "?")
        off = self.reader.board_fact("predicted_offset_mv")
        bits = [f"board {cfg_name}, {board_cpu:,.1f} counts/{self.cfg['unit_small']}"]
        if r2 is not None and r4 is not None:
            bits.append(f"comp {r2:g}/{r4:g} ohm")
        if off is not None:
            bits.append(f"predicted offset {off:+.2f} mV")
        if fs:
            bits.append(f"HX711 gain {round(128 * self._gain_ratio())} (+/-{fs:.1f} mV)")

        problems = []
        if abs(board_cpu - host_cpu) / max(abs(board_cpu), 1e-9) > 0.001:
            problems.append(
                f"host expects {host_cpu:,.1f} counts/{self.cfg['unit_small']}. Fix "
                "CONFIG / POISSON_RATIO / GAUGE_FACTOR in the dashboard so they "
                "match the sketch, or the microstrain numbers are wrong.")
        exp = self.cfg.get("comp_expected")
        if exp is not None and r2 is not None and r4 is not None and \
           (abs(r2 - exp) > 0.5 or abs(r4 - exp) > 0.5):
            problems.append(
                f"host expects {exp:g} ohm completion. Click '{exp:g} ohm' if "
                "that is what is fitted, or pick the other button.")
        if r2 is not None and r4 is not None and abs(r2 - r4) > 0.5 and \
           cfg_name in COMPLETION_CONFIGS:
            problems.append("R2 and R4 differ. A mixed completion pair rails the "
                            "HX711; fit a matched pair.")

        if problems:
            self.match_var.set("MISMATCH: " + "  ".join(bits) + "\n" +
                               "\n".join(problems))
            self.match_lbl.config(fg="#c62828")
        else:
            self.match_var.set("host and board agree: " + "  ".join(bits))
            self.match_lbl.config(fg="#2e7d32")
        self._mismatch_checked = True

    # ------------------------------------------------------------ actions
    def _send(self, cmd):
        self.reader.send(cmd)
        self.capture_status.set(f"sent '{cmd}'")

    def _send_entry(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self._send(cmd)
            self.cmd_entry.delete(0, tk.END)

    # ------------------------------------------------------------ views
    def _mark_view(self):
        key = self.cfg.get("view_key")
        views = self.cfg.get("views", {})
        for k, b in getattr(self, "view_btns", {}).items():
            on = (k == key)
            b.config(relief=tk.SUNKEN if on else tk.RAISED,
                     bg="#1565c0" if on else "#e0e0e0", fg="white" if on else "black")
        if key in views:
            v = views[key]
            self.view_info.set(f"{v['label']}:  {v['config_name']}, "
                               f"{v['counts_per_unit']:.1f} counts/ustrain, calibration file "
                               f"{Path(v['calib_file']).name}")

    def _switch_view(self, key):
        """Send 'x <n>' and swap everything that depends on the view."""
        views = self.cfg.get("views", {})
        if key not in views or key == self.cfg.get("view_key"):
            return
        if self.capture_buffer is not None or self.pair_state is not None:
            self.capture_status.set("Finish or cancel the capture before switching gauges.")
            return
        if self.points and not tk.messagebox.askyesno(
                "Switch gauges", "Captured points belong to this view and will be cleared. Switch?"):
            return
        v = views[key]
        self.cfg.update(view_key=key, counts_per_unit=v["counts_per_unit"],
                        calib_file=v["calib_file"], scale_label=v["scale_label"],
                        default_scale=v["default_scale"])
        self.root.title(f"{self.cfg['title_base']} - {v['label']}")
        self.reader.forget_fact("counts_per_ustrain", "bridge_factor", "config", "view",
                                "saved_scale_view", "saved_scale", "saved_offset")
        self.cfg["final_file"] = None if self.cfg.get("no_final") else v.get("final_file")
        self.reader.send(v["x_cmd"])
        self.points.clear()
        self._refresh_tree()
        self._recompute_fit()
        self.calibrated = False
        self.fitted_scale = None
        self.raw_buffer.clear()
        self.t_hist.clear()
        self.v_hist.clear()
        self._load_final()                 # this view's accepted calibration, if any
        self._mark_view()
        self._restore_session_calibration()
        self.fit_var.set(self.fit_var.get() if self.calibrated else
                         "Capture at least 2 points.")
        self.capture_status.set(f"Switched to {v['label']}. "
                                + ("Calibrated earlier this session." if self.calibrated
                                   else "Not calibrated this session yet."))

    def _two_bridge_cmd(self, cmd):
        """Two-bridge commands, with a confirm on the ones that change settings."""
        from tkinter import messagebox
        if cmd in ("i 1", "i 2") and not messagebox.askyesno(
                "Flip sign", f"Flip the sign of {'CH1 middle' if cmd == 'i 1' else 'CH2 left + right'}?\n"
                "Only if loading that bridge makes it read DOWN. Re-zero and recalibrate after."):
            return
        self.reader.send(cmd)
        hints = {"v": "Both bridges printed in Board messages.",
                 "i 1": "CH1 sign flipped (saved). SAVE ZERO, then recalibrate.",
                 "i 2": "CH2 sign flipped (saved). SAVE ZERO, then recalibrate.",
                 "g m": "Middle captured. Move the SAME mass to a side strip, then click 2.",
                 "g s": "Side weight g saved (see Board messages). Recalibrate 'All 6'.",
                 "a 3": "Zero tracking ON: with the IMU, the zero follows while tipped back "
                        "and still (steps up to 3 kg).",
                 "a 0": "Zero tracking OFF: use this while watching drift by eye."}
        self.capture_status.set(hints.get(cmd, f"sent '{cmd}'"))

    def _sim_pose(self, m):
        """Simulated pose 'q f s|r|t|0' (env:loadcell_test only). Turns pose
        gating on first if it is off, since the simulated pose only matters
        with gating."""
        if m != "0" and self.reader.board_fact("pose_gating") is False:
            self.reader.send("q 1")
        self.reader.send(f"q f {m}")

    def _set_imu_mode(self, use_imu):
        """'q 1' = pose gating with the IMU, 'q 0' = no IMU: every reading is
        treated as standing (live). Saved on the board; it reprints its IMU block,
        which updates the mode label."""
        self.reader.forget_fact("pose_gating")
        self.reader.send("q 1" if use_imu else "q 0")
        if use_imu:
            self.capture_status.set(
                "IMU mode ON: zero tracks while tipped back, readings only when standing. "
                "Without an IMU fitted, assist stays 0 (IMU FAULT).")
        else:
            self.capture_status.set(
                "No-IMU mode: every reading is treated as standing. Zero by hand: TARE with the "
                "plate empty and the trolley tipped back, then stand it up with the load and let go.")

    def _update_imu_mode(self):
        gating = self.reader.board_fact("pose_gating")
        found = self.reader.board_fact("imu_found")
        if gating is None:
            text, fg = "mode: unknown (send 'b')", "#777"
        elif not gating:
            text, fg = "NO IMU - treated as standing", "#1565c0"
        elif found is False:
            text, fg = "IMU mode, but NO IMU FOUND - assist 0", "#c62828"
        else:
            text, fg = "IMU mode - zero tipped back, read standing", "#2e7d32"
        self.imu_mode_var.set(text)
        self.imu_mode_lbl.config(fg=fg)

    def _set_comp(self, ohms):
        """Tell the board which completion resistors are fitted (saved in its EEPROM)."""
        self.cfg["comp_expected"] = ohms
        self.reader.send(f"r {ohms:g}")
        self.capture_status.set(
            f"Sent 'r {ohms:g}'. The board reprints the bridge report and runs a "
            f"diagnostic (see Board messages). Check measured offset against "
            f"predicted, let it settle, then TARE. Sensitivity is unchanged, but "
            f"recapture points if the bridge was disturbed during the swap.")

    def _do_tare(self):
        self.reader.send("t")
        self.capture_status.set("Tare sent.")

    def _read_known(self):
        text = self.known_entry.get().strip()
        try:
            return float(text)
        except ValueError:
            self.capture_status.set("Enter a number first.")
            return None

    # -- single shot, kept for a quick look -------------------------------
    def _capture_point(self):
        known = self._read_known()
        if known is None:
            return
        if self.capture_buffer is not None or self.pair_state is not None:
            self.capture_status.set("Already capturing, wait...")
            return
        if not self._pose_ok(POSE_STANDING, "Stand it up with the load on, let go, wait, then capture."):
            return
        self.capture_target = known
        self.capture_buffer = []
        self.capture_status.set(
            f"Single capture, {AVERAGE_SAMPLES} samples. This is referenced to "
            f"the last zero, so any drift since then lands in the fit.")

    # -- paired, drift safe ------------------------------------------------
    def _start_pair(self):
        if self.pair_state == "await_load":
            if not self._pose_ok(POSE_STANDING, "Stand it up, let go, wait ~10 s, then click Continue."):
                return
            self.pair_state = "load"
            self.capture_buffer = []
            self.pair_btn.config(text="capturing load...")
            self.capture_status.set(f"Capturing LOADED, {AVERAGE_SAMPLES} samples...")
            return
        if self.pair_state is not None or self.capture_buffer is not None:
            self.capture_status.set("Already capturing, wait...")
            return

        known = self._read_known()
        if known is None:
            return
        if not self._pose_ok(POSE_TIPPED_STILL, "Empty the plate, let it rest tipped back, "
                                               "hands off, then click again."):
            return
        self.capture_target = known
        self.pair_state = "zero"
        self.capture_buffer = []
        self.pair_btn.config(text="capturing zero...")
        self.capture_status.set(
            f"Plate EMPTY, tipped back, hands off. Capturing ZERO, {ZERO_SAMPLES} samples...")

    def _finish_zero(self):
        self.pair_zero = statistics.mean(self.capture_buffer)
        self.capture_buffer = None
        self.pair_state = "await_load"
        self.pair_btn.config(text="Continue: load placed")
        self.capture_status.set(
            f"Zero = {self.pair_zero:,.0f} counts. Stand it up, place "
            f"{self.capture_target:g} {self.in_unit} (the TOTAL on the plate), let go, "
            f"wait ~10 s, then click Continue.")

    def _finish_pair(self):
        loaded = statistics.mean(self.capture_buffer)
        spread = max(self.capture_buffer) - min(self.capture_buffer)
        counts = loaded - self.pair_zero
        self._log_point(self.capture_target, counts, "pair", spread, loaded, self.pair_zero)
        self.points.append((self.capture_target, counts, "pair"))
        self.points.sort(key=lambda p: p[0])
        self._refresh_tree()
        self.capture_status.set(
            f"Captured {self.capture_target:g} {self.in_unit} = "
            f"{counts:,.0f} counts  (loaded {loaded:,.0f} minus zero "
            f"{self.pair_zero:,.0f}, spread {spread:,.0f})")
        self.capture_buffer = None
        self.pair_state = None
        self.pair_zero = None
        self.capture_target = None
        self.pair_btn.config(text="Capture zero + load")
        self.known_entry.delete(0, tk.END)
        self._recompute_fit()

    def _finish_capture(self):
        counts = statistics.mean(self.capture_buffer)
        spread = max(self.capture_buffer) - min(self.capture_buffer)
        self._log_point(self.capture_target, counts, "single", spread, counts, None)
        self.points.append((self.capture_target, counts, "single"))
        self.points.sort(key=lambda p: p[0])
        self._refresh_tree()
        self.capture_status.set(
            f"Captured {self.capture_target:g} {self.in_unit} "
            f"= {counts:,.0f} counts (spread {spread:,.0f})")
        self.capture_buffer = None
        self.capture_target = None
        self.known_entry.delete(0, tk.END)
        self._recompute_fit()

    def _refresh_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for known, counts, how in self.points:
            self.tree.insert("", "end",
                             values=(f"{known:g}", f"{counts:,.0f}", how))

    def _delete_point(self):
        sel = self.tree.selection()
        if not sel:
            return
        del self.points[self.tree.index(sel[0])]
        self._refresh_tree()
        self._recompute_fit()

    def _clear_points(self):
        self.points.clear()
        self._refresh_tree()
        self._recompute_fit()

    def _set_apply(self, state):
        self.apply_btn.config(state=state)
        self.apply_btn_quick.config(state=state)

    def _recompute_fit(self):
        xs_in = [p[0] for p in self.points]                 # as typed (e.g. g)
        xs = [x * self.in_to_unit for x in xs_in]           # fit unit (e.g. N)
        ys = [p[1] for p in self.points]
        self.fit_pts.set_data(xs_in, ys)
        if len(self.points) >= 2:
            m, c, r2 = linear_fit(xs, ys)
            if m is None or m == 0:
                self.fit_var.set("Points are degenerate (all same known value).")
                self._set_apply(tk.DISABLED)
                return
            self.fitted_scale, self.fitted_offset = m, c
            self.fitted_r2 = r2
            span_in = [min(0.0, min(xs_in)), max(xs_in)]       # show where the line meets zero load
            self.fit_line.set_data(span_in,
                                   [m * x * self.in_to_unit + c for x in span_in])
            residuals = [y - (m * x + c) for x, y in zip(xs, ys)]
            self.res_pts.set_data(xs_in, residuals)
            max_res = max(abs(r) for r in residuals)

            guess = self.cfg["default_scale"] * self._gain_ratio()
            ratio = m / guess if guess else float("nan")
            per_in = (f"counts per {self.in_unit:<3}    = {m * self.in_to_unit:,.4f}\n"
                      if self.in_unit != self.cfg["unit"] else "")
            txt = (f"counts per {self.cfg['unit']:<3}    = {m:,.2f}   (sent to board)\n"
                   + per_in +
                   f"offset (counts)     = {c:,.1f}   (sent to board)\n"
                   f"zero reads as       = {self._offset_kg(m, c):+.2f} kg\n"
                   f"R^2                 = {r2:.6f}\n"
                   f"max residual        = {max_res:,.0f} counts\n"
                   f"vs starting guess   = {ratio:.2f}x")

            warn = []
            if r2 < 0.99:
                warn.append("R^2 < 0.99: on a stiff beam this is a fault, not a\n"
                            "  result. Suspect contact resistance in a bridge arm.")
            if len(self.points) < 3:
                warn.append("fewer than 3 points")
            # A large offset is expected here: the trolley is zeroed tipped back but
            # measured standing, and it only stands once the load beats its heavy back.
            if any(p[2] == "single" for p in self.points) and \
               any(p[2] == "pair" for p in self.points):
                warn.append("mixing single and paired points. Pick one method.")
            if warn:
                txt += "\nWARNING: " + "\nWARNING: ".join(warn)
            self.fit_var.set(txt)
            self._set_apply(tk.NORMAL)
        else:
            self.fit_line.set_data([], [])
            self.res_pts.set_data([], [])
            self.fit_var.set("Capture at least 2 points.")
            self._set_apply(tk.DISABLED)

        for ax in (self.ax_fit, self.ax_res):
            ax.relim()
            ax.autoscale_view()

    def _apply_scale(self):
        m, c = self.fitted_scale, self.fitted_offset or 0.0
        cmd = f"{self.cfg['scale_command']} {m:.4f} {c:.1f}"
        self.reader.send(cmd)
        self.calibrated = True
        self.session_scales[self.cfg.get("view_key")] = (m, c)
        try:
            self._save_calibration(m, {"offset_counts": c,
                                       "zero_reads_kg": self._offset_kg(m, c)})
            where = f" and saved to {self._calib_path().name}" if self._calib_path() else ""
        except OSError as e:
            where = f" (could not save file: {e})"
        logged = self._log_apply(m, c)
        self.capture_status.set(
            f"Applied: {cmd}  ({m:,.2f} counts/{self.cfg['unit']}, offset {c:,.0f} counts). "
            f"Stored in the board's flash{where}. It stays until you APPLY again or send 'u'."
            + (f" Logged to {logged}." if logged else ""))

    # ------------------------------------------------------------ data log
    def _log_dir(self):
        f = getattr(self.reader, "filename", None)
        return Path(f).parent if f else None

    def _log_point(self, known, counts, method, spread, loaded, zero):
        """Every captured point, applied or not: <session>_points.csv."""
        f = getattr(self.reader, "filename", None)
        if not f:
            return
        path = Path(f).with_name(Path(f).stem + "_points.csv")
        new = not path.exists()
        try:
            with open(path, "a", newline="") as fh:
                w = csv.writer(fh)
                if new:
                    w.writerow(["time", "view", f"known_{self.in_unit}", "counts", "method",
                                "spread_counts", "loaded_counts", "zero_counts", "pose", "hx711_gain"])
                w.writerow([datetime.now().isoformat(timespec="seconds"), self.cfg.get("view_key"),
                            known, f"{counts:.1f}", method, f"{spread:.0f}",
                            f"{loaded:.1f}", "" if zero is None else f"{zero:.1f}",
                            self._pose_now(), round(128 * self._gain_ratio())])
        except OSError:
            pass

    def _log_apply(self, m, c):
        """Each APPLY: one line in calibration_history.csv plus a full JSON record
        (fit + every point) next to the session CSV. Returns the history file name."""
        d = self._log_dir()
        if not d:
            return None
        stamp = datetime.now()
        session = Path(self.reader.filename).stem
        try:
            hist = d / "calibration_history.csv"
            new = not hist.exists()
            with open(hist, "a", newline="") as fh:
                w = csv.writer(fh)
                if new:
                    w.writerow(["time", "session", "view", "counts_per_N", "offset_counts",
                                "zero_reads_kg", "r2", "n_points", "methods", "hx711_gain",
                                "points (known:counts)"])
                w.writerow([stamp.isoformat(timespec="seconds"), session, self.cfg.get("view_key"),
                            f"{m:.4f}", f"{c:.1f}", f"{self._offset_kg(m, c):+.3f}",
                            f"{getattr(self, 'fitted_r2', float('nan')):.6f}", len(self.points),
                            "/".join(sorted({p[2] for p in self.points})),
                            round(128 * self._gain_ratio()),
                            " ".join(f"{k:g}:{n:.0f}" for k, n, _ in self.points)])
            rec = {"time": stamp.isoformat(timespec="seconds"), "session_csv": Path(self.reader.filename).name,
                   "view": self.cfg.get("view_key"), "counts_per_N": m, "offset_counts": c,
                   "zero_reads_kg": self._offset_kg(m, c), "r2": getattr(self, "fitted_r2", None),
                   "hx711_gain": round(128 * self._gain_ratio()), "input_unit": self.in_unit,
                   "points": [{"known": k, "counts": n, "method": how} for k, n, how in self.points]}
            (d / f"{session}_applied_{stamp:%H%M%S}.json").write_text(json.dumps(rec, indent=2))
            return hist.name
        except OSError:
            return None

    def _save_zero(self):
        self.reader.send("z")
        self.capture_status.set(
            "Sent 'z': the board averages ~4 s and saves it as the permanent zero "
            "(used at every power-up). Plate EMPTY, tipped back, hands off.")

    # ------------------------------------------------------------ assist
    STATUS_NAMES = {0: "OK", 1: "SETTLING", 2: "NO CALIBRATION", 3: "SENSOR RAILED",
                    4: "SIGNAL FROZEN", 5: "NO DATA", 6: "IMPLAUSIBLE READING",
                    7: "OVERLOAD (assist capped)", 8: "HELD (not standing)",
                    9: "IMU FAULT"}

    def _update_assist(self, s):
        status = s.get("status", float("nan"))
        if status != status or not s.get("connected"):      # NaN: old sketch
            self.assist_state_var.set("payload: no assist data (needs sketch v3.6)")
            self.assist_state_lbl.config(fg="#777")
            self.assist_big_var.set("--")
            self.load_pct_var.set("load: --")
            self._draw_load_gauge(None, None)
            self.assist_sub_var.set("")
            return
        status = int(status)
        valid = s.get("valid", 0) >= 0.5
        akg = s.get("assist_kg", 0.0)
        apct = s.get("assist_pct", 0.0)
        name = self.STATUS_NAMES.get(status, f"status {status}")
        measured_kg = s[self.cfg["value_field"]] / 9.80665
        rated = self.reader.board_fact("rated_kg")
        sf = self.reader.board_fact("assist_sf", 0.7)

        if status == 8:
            self.assist_state_var.set("payload HELD - not standing, assist from the last standing reading")
            fg = "#6a1b9a"
        elif valid:
            self.assist_state_var.set(f"payload VALID - {name}")
            fg = "#ef6c00" if status == 7 else "#2e7d32"
        elif status == 1:
            self.assist_state_var.set("payload SETTLING - assist can only drop")
            fg = "#ef6c00"
        else:
            self.assist_state_var.set(f"payload INVALID - {name} - assist OFF")
            fg = "#c62828"
        self.assist_state_lbl.config(fg=fg)
        self.assist_big_lbl.config(fg=fg)
        self.assist_big_var.set(f"assist {apct:5.1f} %   ({akg:.1f} kg)")
        rated_kg = rated or 150.0
        load_pct = measured_kg / rated_kg * 100.0
        self.load_pct_var.set(f"load {load_pct:5.1f} % of rated   "
                              f"({measured_kg:.1f} / {rated_kg:.0f} kg)")
        self._draw_load_gauge(load_pct, apct if valid or status == 1 else 0.0)
        rated_txt = f"{rated:.1f} kg" if rated else "?"
        p = self._pose_now()
        if p:
            pose_txt = f"{POSE_NAMES.get(p, p)}, tilt {s.get('tilt_deg', 0.0):.1f} deg"
        elif self.reader.board_fact("pose_gating") is False:
            pose_txt = "no-IMU mode: treated as standing"
        else:
            pose_txt = "no IMU data"
        self.assist_sub_var.set(
            f"pose         : {pose_txt}\n"
            f"measured     : {measured_kg:8.2f} kg\n"
            f"x {sf:.2f}       : {max(0.0, sf * measured_kg):8.2f} kg  "
            f"(assist sized below the load on purpose)\n"
            f"rated payload: {rated_txt}   (set on board with 'p <kg>')")

    GAUGE_MAX_PCT = 125.0     # bar runs past 100 % so overload is visible

    def _draw_load_gauge(self, load_pct, assist_pct):
        """Horizontal bar: measured load as % of rated payload, with the
        assist level (0.9 x load) marked on the same scale."""
        c = self.load_canvas
        c.delete("all")
        w = max(c.winfo_width(), 300)
        x0, x1 = 10, w - 10
        top, bot = 8, 38
        span = x1 - x0
        to_x = lambda pct: x0 + span * max(0.0, min(pct, self.GAUGE_MAX_PCT)) / self.GAUGE_MAX_PCT

        x100 = to_x(100.0)
        c.create_rectangle(x0, top, x1, bot, outline="#999", fill="#f2f2f2")
        c.create_rectangle(x100, top, x1, bot, outline="", fill="#fbe3e3")   # overload zone

        if load_pct is not None:
            if load_pct > 100.0:
                fill = "#c62828"
            elif load_pct > 85.0:
                fill = "#ef6c00"
            else:
                fill = "#1565c0"
            c.create_rectangle(x0, top, to_x(load_pct), bot, outline="", fill=fill)
            if assist_pct is not None and assist_pct > 0:
                xa = to_x(assist_pct)
                c.create_rectangle(x0, bot - 7, xa, bot, outline="", fill="#2e7d32")
                c.create_line(xa, top - 4, xa, bot + 4, fill="#2e7d32", width=2)
            if load_pct > self.GAUGE_MAX_PCT:
                c.create_text(x1 - 4, (top + bot) / 2, text=">125%", anchor="e",
                              fill="white", font=("Segoe UI", 9, "bold"))

        c.create_rectangle(x0, top, x1, bot, outline="#555")
        c.create_line(x100, top - 4, x100, bot + 4, fill="#c62828", width=2)
        for pct in (0, 25, 50, 75, 100, 125):
            x = to_x(pct)
            c.create_line(x, bot, x, bot + 5, fill="#555")
            c.create_text(x, bot + 14, text=f"{pct}%", fill="#555",
                          font=("Segoe UI", 8))
        c.create_text(x0, bot + 28, anchor="w", fill="#555", font=("Segoe UI", 8),
                      text="bar = measured load    green = assist (0.9 x load)    "
                           "red line = rated payload")

    # ------------------------------------------------------------ loop
    def _tick(self):
        s = self.reader.snapshot()
        value = s[self.cfg["value_field"]]
        raw_tared = s[self.cfg["raw_field"]]
        raw_abs = s.get("raw", raw_tared)

        # only log genuinely new samples
        is_new = (s["t_ms"] != self._last_t)
        if is_new:
            self._last_t = s["t_ms"]
            self.raw_buffer.append(raw_abs)

        label, col = connection_status(s)
        htext, hcolour, hdetail = self._assess_health(s, label)
        self.health_var.set(htext)
        self.health_frame.config(bg=hcolour)
        self.health_lbl.config(bg=hcolour)
        self.health_detail.set(hdetail)

        self._check_board_match()
        self._update_imu_mode()
        self._sync_final()

        warn = self.cfg.get("warn_above")
        colour = "#c62828" if (warn and abs(value) > warn) else "#1565c0"

        big_scale = self.cfg.get("big_scale", 1.0)
        big_unit = self.cfg.get("big_unit", self.cfg["unit"])
        big_value = value * big_scale

        if htext in ("SENSOR RAILED", "SIGNAL FROZEN", "NO DATA FROM BOARD"):
            self.big_var.set("---")
            self.big_lbl.config(fg="#c62828")
            self.big_sub_var.set("reading invalid - see banner above")
            self.big_sub_lbl.config(fg="#c62828")
        else:
            self.big_var.set(f"{big_value:+,.1f} {big_unit}")
            self.big_lbl.config(fg=colour)
            sub = f"{value:+.3f} {self.cfg['unit']}"
            if not self.points and not self.calibrated:
                sub += "   - UNCALIBRATED, scale is a placeholder"
                self.big_sub_lbl.config(fg="#ef6c00")
            else:
                self.big_sub_lbl.config(fg="#555")
            self.big_sub_var.set(sub)

        if self.assist_on:
            self._update_assist(s)

        detail = [f"tared counts : {raw_tared:>14,.0f}"]
        for lbl, key, fmt in self.cfg["extra_fields"]:
            detail.append(f"{lbl:<13}: {format(s[key], fmt):>14}")
        self.detail_var.set("\n".join(detail))

        if is_new:
            self.live_log.insert(
                tk.END,
                f"{s['t_ms']:>9.0f}  raw={raw_abs:>12,.0f}  "
                f"tared={raw_tared:>10,.0f}  {value:>10.3f} {self.cfg['unit']}\n")
            self.live_log.see(tk.END)
            if float(self.live_log.index('end-1c').split('.')[0]) > 400:
                self.live_log.delete("1.0", "150.0")

        # capture state machine
        if self.capture_buffer is not None and is_new:
            self.capture_buffer.append(raw_tared)
            got = len(self.capture_buffer)
            if self.pair_state == "zero":
                if got >= ZERO_SAMPLES:
                    self._finish_zero()
                else:
                    self.capture_status.set(f"ZERO... {got}/{ZERO_SAMPLES}")
            elif self.pair_state == "load":
                if got >= AVERAGE_SAMPLES:
                    self._finish_pair()
                else:
                    self.capture_status.set(f"LOADED... {got}/{AVERAGE_SAMPLES}")
            else:
                if got >= AVERAGE_SAMPLES:
                    self._finish_capture()
                else:
                    self.capture_status.set(f"Capturing... {got}/{AVERAGE_SAMPLES}")

        now = time.time()
        self.t_hist.append(now)
        self.v_hist.append(value)
        if len(self.t_hist) > 1:
            rel = [t - self.t_hist[-1] for t in self.t_hist]
            self.live_line.set_data(rel, list(self.v_hist))
            self.ax_live.set_xlim(-HISTORY_SECONDS, 0)
            lo, hi = min(self.v_hist), max(self.v_hist)
            pad = max(abs(hi - lo) * 0.2, 0.5)
            self.ax_live.set_ylim(lo - pad, hi + pad)

        for line in self.reader.pop_log_lines():
            self.log.insert(tk.END, line + "\n")
            self.log.see(tk.END)

        self.status_var.set(
            f"t={s['t_ms']:>8.0f} ms  rate={s['rate_hz']:5.1f} Hz  {label}")
        self.status_lbl.config(fg=col)

        self.canvas.draw_idle()
        self.root.after(UPDATE_MS, self._tick)

    def _on_close(self):
        self.reader.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        print(f"\nStopped. Saved to {self.reader.filename}")

"""
Strain gauge calibration dashboard for the NUCLEO-G491RE (firmware v4.1).
STM copy of strain_dashboard.py. Uses stm_bench_common.py and stm_calib_gui.py.

Run it
  python stm_dashboard.py                    start window: port, test name, gauges
  python stm_dashboard.py COM7 test_30       straight in, all 6 gauges (combined)
  python stm_dashboard.py COM7 test_30 --view 2
Double-clicking the file, or Run in VS Code, opens the start window.

Gauge views (switchable live with the "Gauges shown" buttons):
  1  middle pair            2 SG   POISSON_HALF   stm_calibration_middle.json
  2  left + right pairs     4 SG   POISSON_FULL   stm_calibration_full.json
  0  all 6, CH1 + g x CH2   6 SG   POISSON_HALF   stm_calibration_combined.json
Each view keeps its own scale + offset on the board, and it is KEPT across
launches (RotaTruck: the assist uses it). The JSON file is written on APPLY as
a record only. Send 'u' from the command box to clear every view on purpose.

Workflow: warm up 20-40 min -> SAVE ZERO (plate empty) -> check signs (press
each strip, the reading should rise; flip a sign if not) -> calibrate the
view with known masses -> APPLY. For "All 6": balance g first (mass on the
middle strip, then the same mass on a side strip), then calibrate.
"""

import argparse
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# ---- must match the firmware -------------------------------------------
POISSON_RATIO = 0.33          # aluminium
GAUGE_FACTOR = 2.0
R_GAUGE = 350.0
COMP_CHOICES = (330.0, 350.0) # CH1 completion resistors on hand
R_COMP_DEFAULT = 350.0
HX_AVDD = 3.2                 # HX711 VCC from the Nucleo 3V3
HX_FS_MV = 0.5 * HX_AVDD / 128 * 1000.0
BAUD_DEFAULT = 115200
# ------------------------------------------------------------------------

VIEWS = {
    #       button text              label                          config          x cmd   calibration file
    "0": ("All 6 SG (combined)",    "All 6 gauges, combined",      "poisson_half", "x 0", "stm_calibration_combined.json"),
    "1": ("Middle 2 SG",            "Middle pair (2 gauges)",      "poisson_half", "x 1", "stm_calibration_middle.json"),
    "2": ("Left + right 4 SG",      "Left + right pairs (4 gauges)", "poisson_full", "x 2", "stm_calibration_full.json"),
}
SETTINGS_FILE = HERE / "stm_dashboard_last.json"


def bridge_factor(config):
    return {"poisson_half": (1.0 + POISSON_RATIO, "Poisson half bridge"),
            "poisson_full": (2.0 * (1.0 + POISSON_RATIO), "Poisson full bridge")}[config]


def view_config(key):
    button, label, config, x_cmd, calib = VIEWS[key]
    factor, cname = bridge_factor(config)
    cpu = factor * (GAUGE_FACTOR / 4) * 256 * 8.388608
    return {
        "button": button, "label": label, "config": config, "config_name": cname,
        "x_cmd": x_cmd, "calib_file": str(HERE / calib), "counts_per_unit": cpu,
        "default_scale": cpu / 0.05,
        "scale_label": "Slope is counts per newton. The board converts it with\n"
                       f"{cpu:.1f} counts/ustrain ({cname}, factor {factor:.2f}).",
    }


# ---------------------------------------------------------------- start window
def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [(p.device, p.description) for p in lp.comports()]
    except Exception:
        return []


def next_test_name():
    data_dir = HERE.parent / "data" / "strain_gauge"
    n = 0
    if data_dir.is_dir():
        for f in data_dir.glob("test_*"):
            m = re.match(r"test_(\d+)", f.name)
            if m:
                n = max(n, int(m.group(1)))
    return f"test_{n + 1}"


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


def start_window():
    import tkinter as tk
    from tkinter import ttk, messagebox

    last = load_last()
    result = {}
    root = tk.Tk()
    root.title("STM32 strain gauge dashboard - start")
    root.resizable(False, False)
    frm = ttk.Frame(root, padding=14)
    frm.pack(fill=tk.BOTH, expand=True)
    ttk.Label(frm, text="STM32 strain gauge dashboard", font=("Segoe UI", 13, "bold")).grid(
        row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

    ttk.Label(frm, text="Serial port").grid(row=1, column=0, sticky="w", pady=4)
    port_var = tk.StringVar()
    port_box = ttk.Combobox(frm, textvariable=port_var, width=38)
    port_box.grid(row=1, column=1, sticky="we", pady=4)

    def refresh():
        ports = list_ports()
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
    ttk.Entry(frm, textvariable=test_var, width=40).grid(row=2, column=1, columnspan=2, sticky="we", pady=4)

    ttk.Label(frm, text="Gauges").grid(row=3, column=0, sticky="nw", pady=4)
    view_var = tk.StringVar(value=last.get("view") if last.get("view") in VIEWS else "0")
    vf = ttk.Frame(frm)
    vf.grid(row=3, column=1, columnspan=2, sticky="w", pady=4)
    for key, (_b, label, *_r) in VIEWS.items():
        ttk.Radiobutton(vf, text=label, value=key, variable=view_var).pack(anchor="w")

    ttk.Label(frm, text="CH1 completion R").grid(row=4, column=0, sticky="w", pady=4)
    rcomp_var = tk.StringVar(value=last.get("rcomp", "keep board value"))
    ttk.Combobox(frm, textvariable=rcomp_var, state="readonly", width=20,
                 values=["keep board value"] + [f"{r:g} ohm" for r in COMP_CHOICES]).grid(
        row=4, column=1, sticky="w", pady=4)

    def start(_e=None):
        port = port_var.get().split("  -  ")[0].strip()
        test = test_var.get().strip()
        if not port:
            messagebox.showerror("Missing port", "Pick a serial port (plug the Nucleo in, then Refresh).")
            return
        if not test or re.search(r'[\\/:*?"<>|]', test):
            messagebox.showerror("Test name", "Enter a test name without \\ / : * ? \" < > |")
            return
        rc = rcomp_var.get()
        result.update(port=port, test=test, view=view_var.get(),
                      rcomp=None if rc.startswith("keep") else float(rc.split()[0]))
        save_last({"port": port, "view": view_var.get(), "rcomp": rc})
        root.destroy()

    b = ttk.Frame(frm)
    b.grid(row=5, column=0, columnspan=3, sticky="e", pady=(12, 0))
    ttk.Button(b, text="Cancel", command=root.destroy).pack(side=tk.RIGHT, padx=(6, 0))
    ttk.Button(b, text="Start", command=start).pack(side=tk.RIGHT)
    root.bind("<Return>", start)
    root.mainloop()
    return result or None


# ---------------------------------------------------------------- dashboard
def run_dashboard(port, test_name, view="0", rcomp=None, baud=BAUD_DEFAULT):
    from stm_bench_common import SerialReader
    from stm_calib_gui import CalibrationDashboard

    views = {k: view_config(k) for k in VIEWS}
    v = views[view]
    print(f"# {v['label']}: {v['config_name']}, {v['counts_per_unit']:.1f} counts/ustrain")
    print(f"# port {port}, test '{test_name}', calibration file {Path(v['calib_file']).name}")

    fields = ["raw", "offset_raw", "hx_mV", "bridge_uV", "microstrain", "newtons",
              "valid", "status", "assist_kg", "assist_pct", "pose", "tilt_deg"]
    reader = SerialReader(port, test_name, fields, baud, subdir="strain_gauge")
    reader.start()
    reader.send(v["x_cmd"])                # pick the gauges; queued after the 'b' on connect
    if rcomp is not None:
        reader.send(f"r {rcomp:g}")
    reader.send("m 1")                     # rig standard: stable-hold filter

    title_base = f"STM32 Strain Gauge Calibration - {test_name}"
    config = {
        "title": f"{title_base} - {v['label']}",
        "title_base": title_base,
        "unit": "N",
        "value_field": "newtons",
        "raw_field": "offset_raw",
        "extra_fields": [
            ("raw counts", "raw", ",.0f"),
            ("HX711 in", "hx_mV", ".5f"),
            ("bridge", "bridge_uV", ".1f"),
            ("microstrain", "microstrain", ".2f"),
        ],
        "big_scale": 101.9716,
        "big_unit": "g",
        "input_unit": "g",
        "input_to_unit": 9.80665 / 1000.0,
        "calib_file": v["calib_file"],
        "counts_per_unit": v["counts_per_unit"],
        "unit_small": "ustrain",
        "scale_command": "s",
        "scale_label": v["scale_label"],
        "default_scale": v["default_scale"],
        "warn_above": 200.0,
        "comp_choices": COMP_CHOICES,          # CH1 completion, any view
        "comp_expected": rcomp if rcomp is not None else R_COMP_DEFAULT,
        "assist": True,
        "fs_mv": HX_FS_MV,
        "views": views,
        "view_key": view,
    }
    CalibrationDashboard(reader, config).run()


def main():
    if len(sys.argv) == 1:
        choice = start_window()
        if choice:
            run_dashboard(choice["port"], choice["test"], choice["view"], choice["rcomp"])
        return
    ap = argparse.ArgumentParser(description="STM32 strain gauge dashboard")
    ap.add_argument("port", help="COM7 or /dev/ttyACM0")
    ap.add_argument("test_name")
    ap.add_argument("--view", default="0", choices=list(VIEWS),
                    help="0 all 6 combined, 1 middle 2 SG, 2 left + right 4 SG")
    ap.add_argument("--rcomp", type=float, default=None)
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    a = ap.parse_args()
    run_dashboard(a.port, a.test_name, a.view, a.rcomp, a.baud)


if __name__ == "__main__":
    main()

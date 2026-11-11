"""PlatformIO monitor filter `rtt`: serial monitor over the ST-Link (SEGGER RTT).

The STM32F407G-DISC1's ST-Link virtual COM port is not wired to the F407, so
the firmware prints over RTT instead (src/rtt.c).  When the monitor starts
with `monitor_filters = rtt`, this launches OpenOCD with an RTT server on
localhost:9090 (channel 0); `monitor_port = socket://localhost:9090` then
connects the normal PlatformIO serial monitor to it.  OpenOCD is stopped when
the monitor exits.

Options (env vars):  RTT_PORT (9090), RTT_NO_RESET=1 to attach without
resetting the board (default resets, so the boot output is shown).
"""

import atexit
import ctypes
import os
import signal
import subprocess
import time

from platformio.device.monitor.filters.base import DeviceMonitorFilterBase


def _die_with_parent():
    # Linux: kill OpenOCD if the monitor process dies without cleanup, so it
    # never keeps the ST-Link busy for the next upload.
    try:
        ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except OSError:
        pass


class RttMonitor(DeviceMonitorFilterBase):
    NAME = "rtt"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._proc = None
        if self.NAME in (self.options.get("filters") or []):
            self._start_openocd()

    def _start_openocd(self):
        port = int(os.environ.get("RTT_PORT", "9090"))
        ocd = os.path.join(
            os.path.expanduser(os.environ.get("PLATFORMIO_CORE_DIR", "~/.platformio")),
            "packages", "tool-openocd")
        cmd = [os.path.join(ocd, "bin", "openocd"),
               "-s", os.path.join(ocd, "openocd", "scripts"),
               "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg",
               "-c", "init"]
        if os.environ.get("RTT_NO_RESET") != "1":
            cmd += ["-c", "reset run", "-c", "sleep 300"]
        cmd += ["-c", 'rtt setup 0x20000000 0x20000 "SEGGER RTT"',
                "-c", "rtt start",
                "-c", "rtt server start %d 0" % port]

        log_path = os.path.join(self.project_dir or ".", ".pio", "rtt_openocd.log")
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        log = open(log_path, "w")
        self._proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                      stdin=subprocess.DEVNULL,
                                      preexec_fn=_die_with_parent)
        atexit.register(self._stop)

        deadline = time.time() + 8
        while time.time() < deadline:
            if self._proc.poll() is not None:
                with open(log_path) as f:
                    tail = f.read()[-1500:]
                raise SystemExit(
                    "rtt: OpenOCD exited - is the board plugged in, or is an upload/"
                    "debug session holding the ST-Link?\n" + tail)
            # Watch the log rather than probing the socket: a probe connection
            # would be handed (and lose) the buffered boot output.
            with open(log_path) as f:
                if "for rtt connections" in f.read():
                    print("--- rtt: OpenOCD RTT server on localhost:%d (log: %s)"
                          % (port, log_path))
                    return
            time.sleep(0.1)
        self._stop()
        raise SystemExit("rtt: OpenOCD RTT server did not start, see " + log_path)

    def _stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(3)
            except subprocess.TimeoutExpired:
                self._proc.kill()

    # pass text through unchanged
    def rx(self, text):
        return text

    def tx(self, text):
        return text

#!/usr/bin/env bash
# RTT terminal over the ST-Link (STM32F407G-DISC1 has no usable VCP).
# Resets the board, starts OpenOCD with an RTT server on channel 0, and
# attaches this terminal to it.  Ctrl-C to quit.
#   tools/rtt_monitor.sh            reset the board first (shows boot output)
#   tools/rtt_monitor.sh --attach   attach without resetting
set -euo pipefail
PORT=${RTT_PORT:-9090}
OCD=${OPENOCD:-$HOME/.platformio/packages/tool-openocd}
RESET=(-c "reset run" -c "sleep 300")
[[ "${1:-}" == "--attach" ]] && RESET=()

"$OCD/bin/openocd" -s "$OCD/openocd/scripts" \
    -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init" "${RESET[@]}" \
    -c "rtt setup 0x20000000 0x20000 \"SEGGER RTT\"" \
    -c "rtt start" \
    -c "rtt server start $PORT 0" \
    >/tmp/rtt_openocd.log 2>&1 &
OCD_PID=$!
trap 'kill $OCD_PID 2>/dev/null' EXIT

for _ in $(seq 50); do
    nc -z localhost "$PORT" 2>/dev/null && break
    kill -0 $OCD_PID 2>/dev/null || { cat /tmp/rtt_openocd.log; exit 1; }
    sleep 0.1
done
echo "--- RTT terminal on :$PORT (Ctrl-C to quit; space=pause s=scan c=calibrate) ---"
if command -v socat >/dev/null; then
    socat -,raw,echo=0,escape=0x03 TCP:localhost:"$PORT"
else
    nc localhost "$PORT"
fi

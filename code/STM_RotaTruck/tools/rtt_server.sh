#!/usr/bin/env bash
# RTT server only (no terminal), for the host GUIs in host/:
# they connect to socket://localhost:9090.  Keep this running while the GUI
# is open; Ctrl-C stops it and frees the ST-Link for uploads.
# Close the PlatformIO serial monitor first (it runs its own copy).
#   tools/rtt_server.sh            reset the board first (fresh boot log)
#   tools/rtt_server.sh --attach   connect without resetting
set -euo pipefail
PORT=${RTT_PORT:-9090}
OCD=${OPENOCD:-$HOME/.platformio/packages/tool-openocd}
RESET=(-c "reset run" -c "sleep 300")
[[ "${1:-}" == "--attach" ]] && RESET=()

echo "RTT server on socket://localhost:$PORT - Ctrl-C to stop."
exec "$OCD/bin/openocd" -s "$OCD/openocd/scripts" \
    -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init" "${RESET[@]}" \
    -c "rtt setup 0x20000000 0x20000 \"SEGGER RTT\"" \
    -c "rtt start" \
    -c "rtt server start $PORT 0"

#!/usr/bin/env bash
# One-shot: build the simulator, run a scenario, and visualise it.
#
#   ./run.sh [scenario] [mode] [extra args...]
#     scenario : movevp | curve            (default: movevp)
#     mode     : static | anim | live      (default: anim)
#     extra    : forwarded to the Python visualiser (e.g. --save out.gif)
#
# Examples:
#   ./run.sh curve live
#   ./run.sh movevp anim --save run.gif
#   ./run.sh curve static --save curve.png
#
# "static"/"anim" run the C sim to a CSV then plot it; "live" pipes the running
# C process straight into the animator. Either way, edit dsp/*.c and re-run to
# see the new behaviour.
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
SCN="${1:-movevp}"
MODE="${2:-anim}"
shift $(( $# >= 2 ? 2 : $# ))   # remaining args -> Python

echo ">> building C simulator"
make -s -C "$DIR/sim"

CSV="$DIR/sim/${SCN}_log.csv"

case "$MODE" in
  live)
    echo ">> live: C simulator -> Python (piped)"
    ( cd "$DIR/python" && python3 animate_live.py "$SCN" "$@" )
    ;;
  static)
    echo ">> running C simulator -> $CSV"
    "$DIR/sim/motion_sim" "$SCN" --out "$CSV"
    ( cd "$DIR/python" && python3 visualize.py "$CSV" "$@" )
    ;;
  anim)
    echo ">> running C simulator -> $CSV"
    "$DIR/sim/motion_sim" "$SCN" --out "$CSV"
    ( cd "$DIR/python" && python3 animate.py "$CSV" "$@" )
    ;;
  *)
    echo "unknown mode '$MODE' (use static|anim|live)" >&2
    exit 2
    ;;
esac

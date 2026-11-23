#!/usr/bin/env bash
# Stop everything the launcher started (VS Code status bar "STRIDE: Stop"):
#   scripts/stop_all.sh            the running launch (SIGINT, SIGKILL if it hangs), keep the screen
#   scripts/stop_all.sh --screen   also close the launcher window and the shared screen
set -uo pipefail
ws="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
pgf="$ws/data/screen/launch.pgid"
if [[ -f "$pgf" ]]; then
  pg="$(cat "$pgf")"
  gone() { ! kill -0 -- "-$pg" 2>/dev/null; }
  wait_gone() { for _ in $(seq $(($1 * 2))); do gone && return 0; sleep 0.5; done; return 1; }
  if ! gone; then
    echo "stopping launch (process group $pg)…"
    # ros2 launch / bag first: launch then stops its nodes in order. Signalling every process at
    # once makes launch see lo_node exit before its own shutdown, and respawn it (crash respawn).
    # SIGINT, as Ctrl-C would (on SIGTERM ros2 launch exits at once and orphans its nodes). The
    # launcher starts everything through scripts/run_fg, because a process started in the
    # background by a non-interactive shell begins with SIGINT ignored.
    # (not the recorder: launch forwards the signal to it; a second SIGINT makes it exit code 2)
    leaders="$(pgrep -g "$pg" -f '^[^ ]*python3 [^ ]*ros2 (launch|bag play)' || true)"
    [[ -n "$leaders" ]] && kill -INT $leaders 2>/dev/null
    # a recorder finalising a big bag (index, metadata.yaml) may need longer than the nodes
    wait_gone 15 || { pgrep -g "$pg" -f '^[^ ]*python3 [^ ]*ros2 bag' >/dev/null && wait_gone 60; }
    if ! gone; then
      kill -TERM -- "-$pg" 2>/dev/null
      wait_gone 5 || { kill -KILL -- "-$pg" 2>/dev/null; echo "did not stop: killed"; }
    fi
  fi
  rm -f "$pgf"
fi
# anything that escaped its group (gz server/GUI started by ros2 launch, a stray rviz)
# (patterns anchored at the start of the command line, so a shell that merely mentions them is
# never matched)
pkill -TERM -f '^gz sim ' 2>/dev/null
if [[ "${1:-}" == --screen ]]; then
  pkill -f '^python3 [^ ]*lo_tools/stride_launcher.py' 2>/dev/null
  "$ws/scripts/screen" stop
fi
left="$(pgrep -af '^([^ ]*python3 [^ ]*ros2 launch|[^ ]*/(lo_node|lo_sim_rig|lo_graph_demo|rviz2)( |$)|gz sim )' || true)"
[[ -z "$left" ]] && echo "nothing left running" || { echo "still running:"; echo "$left"; }

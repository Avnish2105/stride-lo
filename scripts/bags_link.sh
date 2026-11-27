#!/usr/bin/env bash
# Prepare data/bags (gitignored), the host path mounted read-only at /data/bags by the
# devcontainer (initializeCommand) and by scripts/dc. Bags are optional:
#   STRIDE_BAGS_DIR set                         -> data/bags links to it
#   ~/GPS-denied_State_estimation_Qudrapad/rosbags exists -> data/bags links to it
#   neither                                     -> data/bags is an empty directory
# A non-empty real data/bags directory (bags copied in by hand) is left alone.
# Idempotent; host side only (no-op inside the container).
set -euo pipefail
[[ -f /.dockerenv ]] && exit 0
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
link="$ws/data/bags"
mkdir -p "$ws/data"

src="${STRIDE_BAGS_DIR:-}"
if [[ -z "$src" && -d "$HOME/GPS-denied_State_estimation_Qudrapad/rosbags" ]]; then
  src="$HOME/GPS-denied_State_estimation_Qudrapad/rosbags"
fi

# an empty real dir is our own earlier fallback: replace it if a source has appeared since
[[ -n "$src" && -d "$link" && ! -L "$link" ]] && rmdir "$link" 2>/dev/null || true
if [[ -d "$link" && ! -L "$link" ]]; then
  echo "bags_link: /data/bags <- $link (local directory, unchanged)"
elif [[ -n "$src" ]]; then
  if [[ ! -d "$src" ]]; then
    echo "bags_link: STRIDE_BAGS_DIR=$src is not a directory" >&2
    exit 1
  fi
  src="$(cd "$src" && pwd)"
  [[ "$src" == "$link" ]] || ln -sfn "$src" "$link"
  echo "bags_link: /data/bags <- $src"
else
  [[ -L "$link" ]] && rm -f "$link"
  mkdir -p "$link"
  echo "bags_link: no bags found; /data/bags is empty (set STRIDE_BAGS_DIR or copy bags into data/bags)"
fi

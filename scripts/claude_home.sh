#!/usr/bin/env bash
# Prepare the host user's Claude Code state, bind-mounted into the dev container by the
# devcontainer (initializeCommand) and by scripts/dc, so login, sessions, plans and memory
# survive container rebuilds. Each user mounts their OWN ~/.claude: no credentials are in git.
#   ~/.claude       created if missing (empty: Claude asks to log in once, then it persists)
#   ~/.claude.json  created as {} if missing (else Docker would create it as a directory)
# Existing files are never touched. Idempotent; host side only (no-op inside the container).
set -euo pipefail
[[ -f /.dockerenv ]] && exit 0
mkdir -p "$HOME/.claude"
chmod 700 "$HOME/.claude" 2>/dev/null || true
if [[ ! -e "$HOME/.claude.json" ]]; then
  (umask 077 && printf '{}\n' > "$HOME/.claude.json")
  echo "claude_home: created $HOME/.claude.json"
fi
echo "claude_home: /home/dev/.claude <- $HOME/.claude"

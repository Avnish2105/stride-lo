#!/usr/bin/env bash
# Entrypoint of every STRIDE image target: source the environment, then run the command.
source /usr/local/bin/stride_env.sh
exec "$@"

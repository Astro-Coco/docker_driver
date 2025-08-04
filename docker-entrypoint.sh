#!/usr/bin/env bash
set -e

# 1) delete any default via end0 (ignore error if not present)
ip route del default dev end0 2>/dev/null || true
echo "[entrypoint] removed default route on end0"

# 2) (optional) dump the default route so you can see it in docker logs
echo "[entrypoint] current default route:"
ip route show default

# 3) exec whatever command was passed to the container
exec "$@"

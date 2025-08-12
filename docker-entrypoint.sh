#!/usr/bin/env bash
set -e

ip route del default dev end0 2>/dev/null || true
echo "[entrypoint] removed default route on end0"

echo "[entrypoint] current default route:"
ip route show default

ip route add 192.168.2.0/24 dev end0
echo "[entrypoint] added route 192.168.2.0/24 → end0"

exec "$@"

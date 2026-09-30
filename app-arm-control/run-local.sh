#!/usr/bin/env bash
#
# Launch the arm-control app locally, letting the Viam CLI handle auth.
#
# It mints a short-lived machine API key with the CLI (so you never paste a
# secret), serves app/ over http, and opens the app pointed at your machine.
#
# Usage:
#   ./run-local.sh --machine-id <machine-id> --host <fqdn>
#
#   # or reuse an existing key instead of minting one:
#   ./run-local.sh --host <fqdn> --key-id <id> --key <secret>
#
# Requires: viam CLI (logged in via `viam login`), python3.
set -euo pipefail

MACHINE_ID="" HOST="" KEY_ID="" KEY="" PORT="8765"
while [ $# -gt 0 ]; do
  case "$1" in
    --machine-id) MACHINE_ID="$2"; shift 2;;
    --host)       HOST="$2";       shift 2;;
    --key-id)     KEY_ID="$2";     shift 2;;
    --key)        KEY="$2";        shift 2;;
    --port)       PORT="$2";       shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

if [ -z "$HOST" ]; then echo "need --host <machine-fqdn>" >&2; exit 2; fi

if [ -z "$KEY_ID" ] || [ -z "$KEY" ]; then
  if [ -z "$MACHINE_ID" ]; then
    echo "need --machine-id (to mint a key) or --key-id/--key (to reuse one)" >&2
    exit 2
  fi
  echo "minting machine API key via viam CLI…" >&2
  OUT=$(viam machines api-key create --machine-id "$MACHINE_ID" \
        --name "arm-control-local $(date +%Y%m%d-%H%M%S)")
  echo "$OUT" >&2
  # tolerate a few output phrasings ("Key ID:", "key id:", etc.)
  KEY_ID=$(printf '%s\n' "$OUT" | grep -oiE 'key[ _-]?id[: ]+[0-9a-f-]{36}' | grep -oE '[0-9a-f-]{36}' | head -1)
  KEY=$(printf '%s\n' "$OUT"    | grep -oiE 'key[ _-]?(value|secret)?[: ]+[0-9a-zA-Z]{20,}' | awk '{print $NF}' | tail -1)
  if [ -z "$KEY_ID" ] || [ -z "$KEY" ]; then
    echo "could not parse key id/secret from CLI output above — rerun with --key-id/--key" >&2
    exit 1
  fi
fi

DIR="$(cd "$(dirname "$0")" && pwd)"
URL="http://localhost:${PORT}/index.html?host=${HOST}&api-key-id=${KEY_ID}&api-key=${KEY}"

echo "serving ${DIR} on :${PORT}" >&2
( cd "$DIR" && python3 -m http.server "$PORT" >/dev/null 2>&1 ) &
SERVER_PID=$!
trap 'kill "$SERVER_PID" 2>/dev/null || true' EXIT
sleep 1

echo "opening: ${URL}" >&2
case "$(uname)" in
  Darwin) open "$URL";;
  Linux)  xdg-open "$URL" >/dev/null 2>&1 || echo "open manually: $URL";;
  *)      echo "open manually: $URL";;
esac

echo "Ctrl-C to stop the local server." >&2
wait "$SERVER_PID"

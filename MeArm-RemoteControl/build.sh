#!/usr/bin/env bash
# arm-web one-click build: go vet + go build (web assets embedded via go:embed).
# Run from the arm-web directory.
set -euo pipefail

cd "$(dirname "$0")"

OUT=arm-web
case "$(uname -s)" in
  *MINGW*|*MSYS*|*CYGWIN*) OUT=arm-web.exe ;;
esac

export GOFLAGS=-mod=mod
export GOPROXY=off
export GOSUMDB=off

echo "[build] go vet ./..."
go vet ./...

echo "[build] go build -o ${OUT} ."
go build -o "${OUT}" .

SIZE=$(stat -c %s "${OUT}" 2>/dev/null || stat -f %s "${OUT}")
echo "[build] OK: ${OUT} - ${SIZE} bytes"
echo "[build] done."

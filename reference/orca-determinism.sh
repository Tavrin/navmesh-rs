#!/usr/bin/env bash
# Default-feature crowd trajectories, separate from the sampled-planner goldens.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
args=(cargo build --locked --release -p navmesh-parity --bin nav-orca)
bin=${CARGO_TARGET_DIR:-$root/target}
if [ -n "${NAV_TARGET:-}" ]; then
    args+=(--target "$NAV_TARGET")
    bin="$bin/$NAV_TARGET"
fi
"${args[@]}"
bin="$bin/release/nav-orca"
[ -f "$bin" ] || bin="$bin.wasm"
read -r -a runner <<< "${NAV_RUNNER:-}"
out=$(mktemp)
trap 'rm -f "$out"' EXIT
"${runner[@]}" "$bin" digest > "$out"
if [ "${1:-}" = --update ]; then
    cp "$out" "$here/orca-determinism.golden"
else
    diff -u "$here/orca-determinism.golden" "$out"
fi
cat "$out"

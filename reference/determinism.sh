#!/usr/bin/env bash
# Cross-platform determinism check: bakes the cases that use only downloaded meshes,
# prints one digest line per record and compares them with reference/determinism.golden.
#   NAV_CORPUS_DIR=<dir> reference/determinism.sh [--update]
# The corpus comes from `reference/corpus/fetch.sh --public-only`. The binary is built
# with the `libm` feature, which is what the determinism contract covers. Without
# `--update` any difference is reported and the exit code is 1.
#
# Other targets: NAV_TARGET selects the cargo target and NAV_RUNNER the command that
# runs the binary, for example (wasm32 under wasmtime; the directories the check reads
# must be mapped, see .github/workflows/ci.yml):
#   NAV_TARGET=wasm32-wasip1 \
#   NAV_RUNNER="wasmtime run --dir $PWD --dir $NAV_CORPUS_DIR --env NAV_CORPUS_DIR=$NAV_CORPUS_DIR" \
#   reference/determinism.sh
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
update=0
if [ "${1:-}" = "--update" ]; then update=1; fi
: "${NAV_CORPUS_DIR:?set NAV_CORPUS_DIR to a directory filled by: reference/corpus/fetch.sh --public-only}"

# Cases whose meshes are downloaded (no oracle needed): solo, tiled, tile cache, crowds.
cases=(dungeon_solo dungeon_tiled dungeon_tilecache nav_test_solo nav_test_tiled nav_test_areas godot_level_solo undulating_crowd)

target=${NAV_TARGET:-}
build=(cargo build --locked --release -p navmesh-parity --bin nav-port --features libm)
bin="${CARGO_TARGET_DIR:-$root/target}"
if [ -n "$target" ]; then
  build+=(--target "$target")
  bin="$bin/$target"
fi
"${build[@]}"
bin="$bin/release/nav-port"
[ -f "$bin" ] || bin="$bin.wasm"
read -r -a runner <<< "${NAV_RUNNER:-}"

out=$(mktemp)
trap 'rm -f "$out"' EXIT
for c in "${cases[@]}"; do
  "${runner[@]}" "$bin" digest "$here/cases/$c.case" >> "$out"
done

if [ "$update" = 1 ]; then
  cp "$out" "$here/determinism.golden"
  echo "wrote $here/determinism.golden ($(wc -l < "$out") lines)"
elif diff -u "$here/determinism.golden" "$out"; then
  echo "determinism: $(wc -l < "$out") record digests match the golden file${target:+ on $target}"
else
  echo "determinism: digests differ from reference/determinism.golden" >&2
  exit 1
fi

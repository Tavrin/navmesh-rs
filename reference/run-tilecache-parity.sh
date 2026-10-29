#!/usr/bin/env bash
# Byte-exact layer stages, obstacle scripts and queries, in both reference widths.
# Run under the build/run admission policy of the calling environment.
set -euo pipefail
out=${1:?usage: run-tilecache-parity.sh <output-directory>}
: "${NAV_CORPUS_DIR:?set NAV_CORPUS_DIR}"
: "${NAV_ORACLE32:?set NAV_ORACLE32}"
: "${NAV_ORACLE64:?set NAV_ORACLE64}"
: "${NAV_PORT32:?set NAV_PORT32}"
: "${NAV_PORT64:?set NAV_PORT64}"
: "${NAV_PARITY:?set NAV_PARITY}"
here=$(cd "$(dirname "$0")" && pwd)
for width in 32 64; do
    oracle=$NAV_ORACLE32; port=$NAV_PORT32; debug=${NAV_DEBUG32:-}
    if [ "$width" = 64 ]; then oracle=$NAV_ORACLE64; port=$NAV_PORT64; debug=${NAV_DEBUG64:-}; fi
    mkdir -p "$out/oracle$width" "$out/port$width" "$out/repeat$width" "$out/debug$width"
    for case in "$here"/cases/*.case; do
        if ! rg -q '^mode = tilecache' "$case"; then continue; fi
        name=$(basename "$case" .case)
        ref="$out/oracle$width/$name.navrec"
        "$oracle" bake "$case" --records "$ref" > "$out/oracle$width/$name.log" 2>&1
        for run in port repeat; do
            "$port" bake "$case" --records "$out/$run$width/$name.navrec" > "$out/$run$width/$name.log" 2>&1
        done
        cmp "$out/port$width/$name.navrec" "$out/repeat$width/$name.navrec"
        "$NAV_PARITY" diff "$ref" "$out/port$width/$name.navrec" > "$out/port$width/$name.diff"
        if [ -n "$debug" ]; then
            "$debug" bake "$case" --records "$out/debug$width/$name.navrec" > "$out/debug$width/$name.log" 2>&1
            "$NAV_PARITY" diff "$ref" "$out/debug$width/$name.navrec" > "$out/debug$width/$name.diff"
        fi
        echo "tilecache $width $name: exact; repeat identical"
    done
done

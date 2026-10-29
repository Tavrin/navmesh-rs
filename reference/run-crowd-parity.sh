#!/usr/bin/env bash
# Compare every enabled crowd case, including full corridors, in both ref widths.
# Supply separately built drivers; this script does not build them.
set -euo pipefail
out=${1:?usage: run-crowd-parity.sh <output-directory>}
: "${NAV_CORPUS_DIR:?set NAV_CORPUS_DIR to the pinned corpus}"
: "${NAV_ORACLE32:?set NAV_ORACLE32 to the 32-bit oracle}"
: "${NAV_ORACLE64:?set NAV_ORACLE64 to the 64-bit oracle}"
: "${NAV_PORT32:?set NAV_PORT32 to nav-port built without default features}"
: "${NAV_PORT64:?set NAV_PORT64 to nav-port built with default features}"
: "${NAV_PARITY:?set NAV_PARITY to nav-parity}"
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out/cases"
out=$(cd "$out" && pwd)
# Keep the checked-in inputs intact while exercising the 300-tick design gate.
for case in "$here"/cases/*.case; do
    if ! rg -q '^crowd_agents = [1-9]' "$case"; then continue; fi
    sed -E 's/^crowd_ticks = .*/crowd_ticks = 300/' "$case" > "$out/cases/$(basename "$case")"
done
for width in 32 64; do
    oracle=$NAV_ORACLE32
    port=$NAV_PORT32
    if [ "$width" = 64 ]; then oracle=$NAV_ORACLE64; port=$NAV_PORT64; fi
    mkdir -p "$out/oracle$width" "$out/port$width" "$out/repeat$width"
    for case in "$out"/cases/*.case; do
        if ! rg -q '^crowd_agents = [1-9]' "$case"; then continue; fi
        name=$(basename "$case" .case)
        ref="$out/oracle$width/$name.navrec"
        "$oracle" bake "$case" --records "$ref" > "$out/oracle$width/$name.log"
        for run in port repeat; do
            "$port" bake "$case" --tiles "$ref" --records "$out/$run$width/$name.navrec" > "$out/$run$width/$name.log"
        done
        cmp "$out/port$width/$name.navrec" "$out/repeat$width/$name.navrec"
        "$NAV_PARITY" diff --crowd "$ref" "$out/port$width/$name.navrec"
    done
done

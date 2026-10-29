#!/usr/bin/env bash
# Bake every case in reference/cases/ with the oracle and write, per case,
# <out-dir>/<case>.navrec (record file) and <out-dir>/<case>.jsonl (summaries).
#   NAV_ORACLE=<nav_oracle binary> NAV_CORPUS_DIR=<corpus dir> run-corpus.sh <out-dir> [case ...]
# Two runs into different directories are compared with
#   cargo run -p navmesh-parity -- diff <out-dir-a> <out-dir-b>
set -euo pipefail

out=${1:?usage: run-corpus.sh <out-dir> [case ...]}
shift
oracle=${NAV_ORACLE:?set NAV_ORACLE to the nav_oracle binary}
: "${NAV_CORPUS_DIR:?set NAV_CORPUS_DIR to the directory filled by reference/corpus/fetch.sh}"
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"

if [ "$#" -gt 0 ]; then
  cases=("$@")
else
  cases=()
  for f in "$here"/cases/*.case; do cases+=("$(basename "$f" .case)"); done
fi

status=0
for name in "${cases[@]}"; do
  start=$(date +%s%N)
  if "$oracle" bake "$here/cases/$name.case" --records "$out/$name.navrec" > "$out/$name.jsonl" 2> "$out/$name.err"; then
    end=$(date +%s%N)
    printf '%-24s ok %8d ms %6d records\n' "$name" "$(( (end - start) / 1000000 ))" "$(wc -l < "$out/$name.jsonl")"
  else
    printf '%-24s FAIL (see %s)\n' "$name" "$out/$name.err"
    status=1
  fi
done
exit $status

#!/usr/bin/env bash
# Fails when tracked or untracked (not ignored) files contain machine paths, personal
# names, internal tool or project names, ticket ids or internal planning labels.
# Untracked files are included so the worktree and CI enforce the same rule.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
# Words are built from parts so this script does not match itself.
private='mo''ss'
owner='etie''nne'
planning='rf''c'
paths='/home/[a-z_][a-z0-9_-]*/|/mnt/[a-z]|/tmp/claude|automation_game'
tools="${private}-(scratch|cargo|heavy|coord|lane|capture|check|mail)|\bapo''llo\b|\blanes?\b|\bcla''ude[0-9]?\b|\bcod''ex\b"
labels="${planning} ?[0-9]|\bS[0-9] oracle\b|\bphase [0-9]+(\.[0-9x]+)?\b|\b${private}-[a-z0-9]{5}\b|\btask-[a-z0-9]{8}\b"
names="\b${private}\b|${owner}|\bdo''ux\b|Legion-Pro"
pattern="${paths}|${tools}|${labels}|${names}"
# Phase labels (P0, P1-RESULTS) are matched case-sensitively: p0/p1 are point names.
phase='\bP[0-9]+(-[A-Z]+)?\b'
matches=""
for args in "-niIE ${pattern}" "-nIE ${phase}"; do
  flags=${args%% *}
  set +e
  found=$(git grep --untracked "$flags" "${args#* }" -- . ':!tools/check_public_text.sh')
  status=$?
  set -e
  case "$status" in
    0) matches="${matches}${found}"$'\n' ;;
    1) ;;
    *) exit "$status" ;;
  esac
done
matches=${matches%$'\n'}
if [ -n "$matches" ]; then
  printf '%s\n' "$matches"
  echo 'FAIL: private machine paths, names or internal labels in public text' >&2
  exit 1
fi
echo 'PASS: public text contains no private machine paths, names or internal labels'

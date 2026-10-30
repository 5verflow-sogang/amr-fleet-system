#!/usr/bin/env bash
# runs.txt의 실행마다 지표 CSV만 받아 data/<라벨>/<반복>/에 둔다. gcloud 로그인 필요
#   experiments/fleet_assignment/fetch.sh [저장 폴더, 기본 experiments/fleet_assignment/data]
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=${1:-$here/data}
grep -v '^#' "$here/runs.txt" | while IFS=', ' read -r label run_id path sha machine; do
  for r in $(gcloud storage ls "$path" | grep '_[0-9][0-9]/$'); do
    k=$(basename "$r" | grep -o '[0-9][0-9]$')
    mkdir -p "$out/$label/$k"
    gcloud storage cp -q "${r}meta.json" "${r}task_outcomes.csv" "${r}assignments.csv" "$out/$label/$k/"
  done
  echo "$label $(ls "$out/$label" | wc -l | tr -d ' ')회"
done

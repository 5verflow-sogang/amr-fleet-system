#!/usr/bin/env bash
# runs.txt의 실행마다 지표 CSV만 받아 data/<라벨>/<반복>/에 둔다 (rosbag·로그는 받지 않는다). gcloud 로그인 필요
#   experiments/research1/central/fetch.sh [저장 폴더, 기본 experiments/research1/central/data]
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=${1:-$here/data}
grep -v '^#' "$here/runs.txt" | while IFS=', ' read -r label run_id path sha machine; do
  for r in $(gcloud storage ls "$path" | grep '_[0-9][0-9]/$'); do
    k=$(basename "$r" | grep -o '[0-9][0-9]$')
    mkdir -p "$out/$label/$k"
    for f in meta.json task_outcomes.csv response_time.csv robot_states.csv fleet_modes.csv; do
      gcloud storage cp -q "$r$f" "$out/$label/$k/" 2>/dev/null || true
    done
  done
  echo "$label $(ls "$out/$label" | wc -l | tr -d ' ')회"
done

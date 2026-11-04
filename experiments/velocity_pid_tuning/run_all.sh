#!/usr/bin/env bash
# 적재 0/10/25 kg x 게인 세트 3종 전부 돌린다 (navigation.md 5-7절 측정).
#   bash experiments/velocity_pid_tuning/run_all.sh <결과 폴더>
set -u
OUT=${1:?결과 폴더}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
rm -f "$OUT/results.csv"

MASSES=(0.0 10.0 25.0)
# label enabled kp ki kd
GAIN_SETS=(
  "off false 0.0 0.0 0.0"
  "default true 0.5 1.0 0.0"
  "aggressive true 1.5 3.0 0.0"
)

for MASS in "${MASSES[@]}"; do
  for GS in "${GAIN_SETS[@]}"; do
    read -r LABEL ENABLED KP KI KD <<< "$GS"
    bash "$HERE/run_one.sh" "$OUT" "$MASS" "$LABEL" "$ENABLED" "$KP" "$KI" "$KD" 0.5
    sleep 1
  done
done

echo "완료: $OUT/results.csv"
cat "$OUT/results.csv"

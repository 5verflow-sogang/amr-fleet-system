#!/bin/bash
# 탐색 실험 2: 트랙 B base 정책(G, 300만 스텝)에서 이어 학습. 다시 실행하면 끝난 것은 건너뛴다.
#   caffeinate -i nice -n 15 scripts/run_info_warm.sh
#
# 갈래: gwarm(G로 이어 학습, 대조군) · i2warm(I2로 이어 학습, 후보) × proposed·r2 × 시드 210~219, 각 150만 스텝.
# 시작점: $BASEM/<비교군>_s<시드>_base_*/ (버킷 research2/tuning_v2/track_b 모델 묶음). 검증·평가는 env_v0 시나리오.
# 평가: 학습 질량 0·2·10·25 kg × S1·S2 × 200 (eval_*), 처음 보는 15·30 kg (eval15_*). 판정: judge_info_pilot.py
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/info_warm}
BASEM=${BASEM:-raw/info_pilot/base_models}
SEEDS=${SEEDS:-$(seq -s ' ' 210 219)}
CANDS=${CANDS:-gwarm i2warm}
VARIANTS=${VARIANTS:-proposed r2}
STEPS=${STEPS:-1500000}
EVAL_EVERY=${EVAL_EVERY:-100000}
RL_EP=${RL_EP:-200}
JOBS=${JOBS:-8}
mkdir -p "$D"
P="$D/progress.txt"
export D P BASEM STEPS EVAL_EVERY RL_EP
log() { echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$P"; }
err() { echo "$(date '+%m-%d %H:%M:%S') $*" >> "$D/errors.txt"; log "오류: $*"; }
export -f log err
log "설정: D=$D SEEDS=[$SEEDS] CANDS=[$CANDS] VARIANTS=[$VARIANTS] STEPS=$STEPS JOBS=$JOBS BASEM=$BASEM"

log "== 0. 사전 점검"
git rev-parse HEAD > "$D/commit.txt"
git rev-parse --abbrev-ref HEAD >> "$D/commit.txt"
git status --porcelain -- . ':!raw' > "$D/dirty.txt"
if [ -s "$D/dirty.txt" ]; then err "커밋 안 된 변경이 있다 (dirty.txt). 중단"; exit 1; fi
shasum -a 256 config/env_v0.toml config/env_v3_info2.toml > "$D/config_sha256.txt"
if ! uv run --frozen --with pytest pytest tests -q > "$D/pytest.log" 2>&1; then err "테스트 실패 (pytest.log). 중단"; exit 1; fi
log "커밋 $(head -1 $D/commit.txt), 테스트 $(tail -1 $D/pytest.log)"

config_of() { case "$1" in gwarm) echo config/env_v0.toml ;; i2warm) echo config/env_v3_info2.toml ;; esac; }
export -f config_of

train_one() {   # $1 = 갈래, $2 = 비교군, $3 = 시드
  if grep -q "^$1 $2 $3 " "$D/runs.txt" 2>/dev/null; then
    dir=$(grep "^$1 $2 $3 " "$D/runs.txt" | tail -1 | cut -d' ' -f4)
    [ -f "$dir/final_model.zip" ] && return 0
  fi
  src=$(ls -d "$BASEM"/"$2"_s"$3"_base_* 2>/dev/null | head -1)
  if [ -z "$src" ]; then err "시작점 없음: $2 시드 $3"; return 0; fi
  logf="$D/train_$1_$2_s$3.log"
  if uv run --frozen python scripts/train_ppo.py --variant "$2" --seed "$3" --steps "$STEPS" \
       --config "$(config_of "$1")" --val-config config/env_v0.toml --tag "$1" --init-from "$src" \
       --eval-every "$EVAL_EVERY" --threads 1 > "$logf" 2>&1; then
    dir=$(grep "^저장:" "$logf" | sed -E 's/^저장: ([^ ]+).*/\1/')
    echo "$1 $2 $3 $dir" >> "$D/runs.txt"
    log "학습 $1 $2 시드 $3 완료"
  else
    err "학습 $1 $2 시드 $3 실패 ($logf)"
  fi
}
export -f train_one
log "== 1. 이어 학습 시작 (동시 $JOBS)"
for s in $SEEDS; do for c in $CANDS; do for v in $VARIANTS; do echo "$c $v $s"; done; done; done |
  xargs -P "$JOBS" -n 3 bash -c 'train_one "$0" "$1" "$2"'
log "== 1. 학습 끝 ($(wc -l < $D/runs.txt)개 기록)"

eval_one() {   # $1 = 접두사, $2 = 질량들, $3 = 갈래, $4 = 비교군, $5 = 시드, $6 = 실행 폴더
  out="$D/$1_$3_$4_s$5"
  [ -f "$out/episodes.csv" ] && return 0
  rm -rf "$out"
  if uv run --frozen python scripts/evaluate.py "$6" --config config/env_v0.toml --masses $2 \
       --episodes "$RL_EP" --mass-obs exact --out-dir "$out" > "$out.log" 2>&1; then
    log "평가 $1 $3 $4 시드 $5 완료"
  else
    err "평가 $1 $3 $4 시드 $5 실패 ($out.log)"
  fi
}
export -f eval_one
log "== 2. 평가 시작 (동시 $JOBS)"
sort -u "$D/runs.txt" | awk '{print "eval", "\"0 2 10 25\"", $0; print "eval15", "\"15 30\"", $0}' |
  xargs -P "$JOBS" -n 6 bash -c 'eval_one "$0" "$1" "$2" "$3" "$4" "$5"'
n_err=$(cat "$D/errors.txt" 2>/dev/null | wc -l | tr -d " ")
log "== 모두 끝. 학습 $(sort -u $D/runs.txt | wc -l), 평가 $(ls $D/eval*/episodes.csv 2>/dev/null | wc -l), 오류 ${n_err}건"
touch "$D/ALL_DONE"

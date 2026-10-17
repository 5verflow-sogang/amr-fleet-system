#!/bin/bash
# 연구 2 v3 3단계 축소 학습: 제약 RL(PPO-Lagrangian, config/env_v3.toml)이 의도대로 움직이는지 본다. 다시 실행하면 끝난 것은 건너뛴다.
#   caffeinate -i scripts/run_v3_pilot.sh
#
# 질문: 충돌 비율을 δ(1%)로 같게 맞췄을 때, 질량을 아는 제안 방법이 가벼울 때 더 빨리 가는가 (R2는 질량을 모른다).
# 비교군 proposed·r2 × 시드 300~302, 100만 스텝. 검증(체크포인트 선택)은 20만 스텝마다 400판, 학습 시나리오(ab) 그대로.
# 평가: ① env_v0 시나리오 S1·S2 × 학습 질량 4개 × 200 (v1·v2와 같은 평가) ② env_v3 학습 분포(mixed) × 학습 질량 4개 × 200
#       (제약 준수 확인: 학습은 확률적 정책, 평가는 결정적 정책이라 충돌 비율이 다를 수 있다)
#
# 판정 기준 (결과 보기 전에 정함. 축소 학습이라 "4단계 튜닝으로 갈지"만 정한다. 시드 3개 평균, 마지막 체크포인트·best 둘 다 본다):
#   P1 제약 수렴: 학습 마지막의 이동평균 충돌 비율(lagrangian/cost_rate_ema)이 비교군마다 시드 3개 중 2개 이상 ≤ 2%
#   P2 λ가 움직임: 학습 마지막 λ가 상한(100)에 붙어 있지 않다 (시드 3개 중 2개 이상)
#   P3 핵심 신호: ② mixed 평가에서 제안의 성공 판 평균 속도가 0 kg이 25 kg보다 5% 이상 빠르고, 그 차이가 R2보다 크다
#   P4 무너지지 않음: ① 평가에서 학습 질량 전체 성공률 ≥ 85% (시드마다)
# P1·P2·P4를 넘고 P3를 넘으면 4단계로. P3만 못 넘으면 원인을 하나씩 본다(학습량 → 시간 벌점 → 신경망). P1·P2를 못 넘으면 PID부터 본다.
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/v3_pilot}
SEEDS=${SEEDS:-300 301 302}
VARIANTS=${VARIANTS:-proposed r2}
STEPS=${STEPS:-1000000}
EVAL_EVERY=${EVAL_EVERY:-200000}
VAL_EP=${VAL_EP:-400}
RL_EP=${RL_EP:-200}
JOBS=${JOBS:-6}
mkdir -p "$D"
P="$D/progress.txt"
export D P STEPS EVAL_EVERY VAL_EP RL_EP
log() { echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$P"; }
err() { echo "$(date '+%m-%d %H:%M:%S') $*" >> "$D/errors.txt"; log "오류: $*"; }
export -f log err   # xargs 안의 bash에서 macOS /usr/bin/log 대신 이 함수를 쓰게
log "설정: D=$D SEEDS=[$SEEDS] VARIANTS=[$VARIANTS] STEPS=$STEPS EVAL_EVERY=$EVAL_EVERY VAL_EP=$VAL_EP RL_EP=$RL_EP JOBS=$JOBS"

# ---------------------------------------------------------------- 0. 사전 점검 (실패하면 시작하지 않는다)
log "== 0. 사전 점검"
git rev-parse HEAD > "$D/commit.txt"
git rev-parse --abbrev-ref HEAD >> "$D/commit.txt"
git status --porcelain -- . ':!raw' > "$D/dirty.txt"
if [ -s "$D/dirty.txt" ]; then err "커밋 안 된 변경이 있다 (dirty.txt). 중단"; exit 1; fi
shasum -a 256 config/env_v3.toml config/env_v0.toml > "$D/config_sha256.txt"
if ! uv run --frozen --with pytest pytest tests -q > "$D/pytest.log" 2>&1; then err "테스트 실패 (pytest.log). 중단"; exit 1; fi
log "커밋 $(head -1 $D/commit.txt), 테스트 $(tail -1 $D/pytest.log)"

# ---------------------------------------------------------------- 1. 학습
train_one() {   # $1 = 비교군, $2 = 시드
  if grep -q "^$1 $2 " "$D/runs.txt" 2>/dev/null; then
    dir=$(grep "^$1 $2 " "$D/runs.txt" | tail -1 | cut -d' ' -f3)
    [ -f "$dir/final_model.zip" ] && return 0
  fi
  logf="$D/train_$1_s$2.log"
  if uv run --frozen python scripts/train_ppo.py --variant "$1" --seed "$2" --steps "$STEPS" \
       --config config/env_v3.toml --tag v3pilot --eval-every "$EVAL_EVERY" --eval-episodes "$VAL_EP" \
       --common-eval-episodes "$VAL_EP" --threads 1 > "$logf" 2>&1; then
    dir=$(grep "^저장:" "$logf" | sed -E 's/^저장: ([^ ]+).*/\1/')
    echo "$1 $2 $dir" >> "$D/runs.txt"
    log "학습 $1 시드 $2 완료 ($(grep '^저장:' "$logf" | sed -E 's/^[^(]*\(//; s/\)$//'))"
  else
    err "학습 $1 시드 $2 실패 ($logf)"
  fi
}
export -f train_one
log "== 1. 학습 시작 (비교군 × 시드, 동시 $JOBS)"
for s in $SEEDS; do for v in $VARIANTS; do echo "$v $s"; done; done |
  xargs -P "$JOBS" -n 2 bash -c 'train_one "$0" "$1"'
log "== 1. 학습 끝 ($(wc -l < $D/runs.txt)개 기록)"

# ---------------------------------------------------------------- 2. 평가 (best 체크포인트)
eval_one() {   # $1 = 비교군, $2 = 시드, $3 = 실행 폴더, $4 = 종류 (v0 | mixed)
  out="$D/eval_$4_$1_s$2"
  [ -f "$out/episodes.csv" ] && return 0
  rm -rf "$out"
  if [ "$4" = v0 ]; then cfg=config/env_v0.toml; sc="S1 S2"; else cfg=config/env_v3.toml; sc=mixed; fi
  if uv run --frozen python scripts/evaluate.py "$3" --config "$cfg" --scenarios $sc --masses 0 2 10 25 \
       --episodes "$RL_EP" --mass-obs exact --out-dir "$out" > "$out.log" 2>&1; then
    log "평가 $4 $1 시드 $2 완료"
  else
    err "평가 $4 $1 시드 $2 실패 ($out.log)"
  fi
}
export -f eval_one
log "== 2. 평가 시작 (동시 $JOBS)"
sort -u "$D/runs.txt" | while read v s dir; do echo "$v $s $dir v0"; echo "$v $s $dir mixed"; done |
  xargs -P "$JOBS" -n 4 bash -c 'eval_one "$0" "$1" "$2" "$3"'
n_err=$(cat "$D/errors.txt" 2>/dev/null | wc -l | tr -d " ")
log "== 모두 끝. 학습 $(sort -u $D/runs.txt | wc -l), 평가 $(ls $D/eval_*/episodes.csv 2>/dev/null | wc -l), 오류 ${n_err}건"
touch "$D/ALL_DONE"

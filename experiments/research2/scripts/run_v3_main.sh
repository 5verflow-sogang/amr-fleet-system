#!/bin/bash
# 연구 2 v3 본 실험 (사전 등록 v3_plan.md). 다시 실행하면 끝난 것은 건너뛰고 이어서 한다.
#   caffeinate -i scripts/run_v3_main.sh
#   uv run python scripts/judge_v3_main.py raw/v3_main
#
# 설정: config/env_v3_final.toml = v2 채택안 ab(충돌 −300, 시간 초과 −75, 돌발 확률 0.9)에서 보상 자르기 한도만 10 → 100.
# 비교군 proposed·r2·r1 × 시드 40~59 (본 실험 범위 0~99: v1 0~19, v2 20~39(예약, 본 실험 안 함), v3 40~59), 300만 스텝.
# 평가: env_v0 시나리오, 질량 0·2·10·25(학습)·15·30(처음 봄) kg × S1·S2 × 200회, 평가 목록 = 학습 시드.
#   제안은 정보 조건 exact·unknown·told0·told25, R1·R2는 질량을 보지 않아 exact만. 그다음 제안의 질량 반응 곡선.
# DWA(B1·B2)는 이진의 코어 수정 뒤 같은 목록 40~59, 같은 평가 설정으로 따로 평가한다 (v3_plan.md 5절).
# 판정 기준은 scripts/judge_v3_main.py와 v3_plan.md 4절 (결과 보기 전에 커밋).
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/v3_main}
SEEDS=${SEEDS:-$(seq -s ' ' 40 59)}
VARIANTS=${VARIANTS:-proposed r2 r1}
CFG=${CFG:-config/env_v3_final.toml}
STEPS=${STEPS:-3000000}
EVAL_EVERY=${EVAL_EVERY:-100000}
RL_EP=${RL_EP:-200}
TRAP_EP=${TRAP_EP:-50}
MASSES=${MASSES:-0 2 10 25 15 30}
JOBS=${JOBS:-6}
mkdir -p "$D"
P="$D/progress.txt"
export D P CFG STEPS EVAL_EVERY RL_EP MASSES
log() { echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$P"; }
err() { echo "$(date '+%m-%d %H:%M:%S') $*" >> "$D/errors.txt"; log "오류: $*"; }
export -f log err   # xargs 안의 bash에서 macOS /usr/bin/log 대신 이 함수를 쓰게
log "설정: D=$D CFG=$CFG SEEDS=[$SEEDS] VARIANTS=[$VARIANTS] STEPS=$STEPS RL_EP=$RL_EP MASSES=[$MASSES] JOBS=$JOBS"

# ---------------------------------------------------------------- 0. 사전 점검 (실패하면 시작하지 않는다)
log "== 0. 사전 점검"
git rev-parse HEAD > "$D/commit.txt"
git rev-parse --abbrev-ref HEAD >> "$D/commit.txt"
git status --porcelain -- . ':!raw' > "$D/dirty.txt"
if [ -s "$D/dirty.txt" ]; then err "커밋 안 된 변경이 있다 (dirty.txt). 중단"; exit 1; fi
shasum -a 256 "$CFG" config/env_v0.toml > "$D/config_sha256.txt"
if ! uv run --frozen --with pytest pytest tests -q > "$D/pytest.log" 2>&1; then err "테스트 실패 (pytest.log). 중단"; exit 1; fi
log "커밋 $(head -1 $D/commit.txt), 테스트 $(tail -1 $D/pytest.log)"
# 보상 검증: 안전 순항 속도(배율 1.0)가 평균 보상 최고
uv run --frozen python scripts/check_reward.py "$CFG" > "$D/check_reward.txt" 2>&1 \
  || { err "check_reward.py 실패 (check_reward.txt). 중단"; exit 1; }
if [ "$(grep -c '가장 높은 배율: 1 ' "$D/check_reward.txt")" -ne 1 ]; then err "보상 검증 통과 못 함 (check_reward.txt). 중단"; exit 1; fi
# 함정 점검: 지나치기·기어가기·안 가기가 정상 주행보다 낮아야 한다
uv run --frozen python scripts/check_reward_traps.py "$CFG" --lists 398 399 --episodes "$TRAP_EP" > "$D/check_traps.txt" 2>&1 \
  || { err "check_reward_traps.py 실패 (check_traps.txt). 중단"; exit 1; }
bad=$(awk '$1 ~ /^(rush|creep|halt)$/ { for (i = 1; i <= NF; i++) if ($i == "대비") { v = $(i + 1); if (v + 0 >= 0) print $0 } }' "$D/check_traps.txt")
if [ -n "$bad" ]; then err "함정 점검 통과 못 함: $bad. 중단"; exit 1; fi
# 규칙 정책 충돌 0건: 돌발 확률 0.9 학습 분포에서도 미리 감속하면 피할 수 있어야 한다
rm -rf "$D/rule_check"
uv run --frozen python scripts/evaluate.py --rule mass_aware --config "$CFG" --eval-list 398 --scenarios mixed \
  --masses 0 2 10 25 --episodes "$RL_EP" --out-dir "$D/rule_check" > "$D/rule_check.log" 2>&1 \
  || { err "규칙 정책 평가 실패 (rule_check.log). 중단"; exit 1; }
read -r n_rule n_rule_col < <(python3 -c 'import csv, sys; r = list(csv.DictReader(open(sys.argv[1])))
print(len(r), sum(x["outcome"] == "collision" for x in r))' "$D/rule_check/episodes.csv")
if [ "$n_rule" -ne $((RL_EP * 4)) ] || [ "$n_rule_col" -ne 0 ]; then err "규칙 정책 점검 실패: ${n_rule}판 중 충돌 ${n_rule_col}건. 중단"; exit 1; fi
log "보상 검증·함정 점검·규칙 정책(${n_rule}판 충돌 0건) 통과"

# ---------------------------------------------------------------- 1. 학습
train_one() {   # $1 = 비교군, $2 = 시드
  if grep -q "^final $1 $2 " "$D/runs.txt" 2>/dev/null; then
    dir=$(grep "^final $1 $2 " "$D/runs.txt" | tail -1 | cut -d' ' -f4)
    [ -f "$dir/final_model.zip" ] && return 0
  fi
  logf="$D/train_$1_s$2.log"
  if uv run --frozen python scripts/train_ppo.py --variant "$1" --seed "$2" --steps "$STEPS" \
       --config "$CFG" --tag final --eval-every "$EVAL_EVERY" --threads 1 > "$logf" 2>&1; then
    dir=$(grep "^저장:" "$logf" | sed -E 's/^저장: ([^ ]+).*/\1/')
    echo "final $1 $2 $dir" >> "$D/runs.txt"
    log "학습 $1 시드 $2 완료"
  else
    err "학습 $1 시드 $2 실패 ($logf)"
  fi
}
export -f train_one
log "== 1. 학습 시작 (비교군 × 시드, 동시 $JOBS)"
for s in $SEEDS; do for v in $VARIANTS; do echo "$v $s"; done; done |
  xargs -P "$JOBS" -n 2 bash -c 'train_one "$0" "$1"'
log "== 1. 학습 끝 ($(wc -l < $D/runs.txt | tr -d ' ')개 기록)"

# ---------------------------------------------------------------- 2. 평가 (env_v0 시나리오, best 체크포인트)
eval_one() {   # $1 = 비교군, $2 = 시드, $3 = 실행 폴더
  out="$D/eval_$1_s$2"
  [ -f "$out/episodes.csv" ] && return 0
  rm -rf "$out"
  if uv run --frozen python scripts/evaluate.py "$3" --config config/env_v0.toml --masses $MASSES \
       --episodes "$RL_EP" --mass-obs exact unknown told0 told25 --out-dir "$out" > "$out.log" 2>&1; then
    log "평가 $1 시드 $2 완료"
  else
    err "평가 $1 시드 $2 실패 ($out.log)"
  fi
}
export -f eval_one
log "== 2. 평가 시작 (동시 $JOBS. R1·R2는 질량을 보지 않아 exact만 돈다)"
sort -u "$D/runs.txt" | awk '{print $2, $3, $4}' | xargs -P "$JOBS" -n 3 bash -c 'eval_one "$0" "$1" "$2"'

# ---------------------------------------------------------------- 3. 질량 반응 곡선 (제안)
log "== 3. 반응 곡선"
uv run --frozen python scripts/probe_v3_mass_response.py "$D/runs.txt" --out "$D" > "$D/probe.log" 2>&1 \
  && log "반응 곡선 완료" || err "반응 곡선 실패 (probe.log)"
n_err=$(cat "$D/errors.txt" 2>/dev/null | wc -l | tr -d " ")
log "== 모두 끝. 학습 $(sort -u $D/runs.txt | wc -l | tr -d ' '), 평가 $(ls $D/eval_*/episodes.csv 2>/dev/null | wc -l | tr -d ' '), 오류 ${n_err}건"
touch "$D/ALL_DONE"

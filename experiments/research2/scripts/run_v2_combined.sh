#!/bin/bash
# 연구 2 v2 합친 후보 튜닝. run_v2_track_a.sh와 같은 구조이고 후보·시드만 다르다. 다시 실행하면 끝난 것은 건너뛰고 이어서 한다.
#   caffeinate -i scripts/run_v2_combined.sh
#
# 후보: base(v1 설정 env_v0) · ab(A2 충돌 −300·시간 초과 −75 + B1 돌발 확률 0.9) × 비교군 proposed·r2 × 시드 220~229.
# 학습 전에 check_reward.py로 안전 순항 속도가 최적인지, 바꾼 학습 시나리오에서 규칙 정책이 충돌 0건인지 확인한다.
# 검증(체크포인트 선택·eval_common)과 평가는 모두 env_v0 시나리오다 (--val-config, evaluate.py --config 기본값).
# 평가: 학습 질량 0·2·10·25 kg × S1·S2 × 200회, 질량 정보 정확. 튜닝 평가 목록 = 학습 시드.
# 채택 규칙은 v2_plan.md(#217) 3절 "합치기"를 따른다 (세 조건 + 개선 폭이 A2·B1 단독보다 큼). 이 스크립트는 판정하지 않는다.
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/v2_combined}
SEEDS=${SEEDS:-$(seq -s ' ' 220 229)}
CANDS=${CANDS:-base ab}
VARIANTS=${VARIANTS:-proposed r2}
STEPS=${STEPS:-3000000}
EVAL_EVERY=${EVAL_EVERY:-100000}
RL_EP=${RL_EP:-200}
JOBS=${JOBS:-6}
mkdir -p "$D"
P="$D/progress.txt"
export D P STEPS EVAL_EVERY RL_EP
log() { echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$P"; }
err() { echo "$(date '+%m-%d %H:%M:%S') $*" >> "$D/errors.txt"; log "오류: $*"; }
export -f log err   # xargs 안의 bash에서 macOS /usr/bin/log 대신 이 함수를 쓰게
log "설정: D=$D SEEDS=[$SEEDS] CANDS=[$CANDS] VARIANTS=[$VARIANTS] STEPS=$STEPS RL_EP=$RL_EP JOBS=$JOBS"

# ---------------------------------------------------------------- 0. 사전 점검 (실패하면 시작하지 않는다)
log "== 0. 사전 점검"
git rev-parse HEAD > "$D/commit.txt"
git rev-parse --abbrev-ref HEAD >> "$D/commit.txt"
git status --porcelain -- . ':!raw' > "$D/dirty.txt"
if [ -s "$D/dirty.txt" ]; then err "커밋 안 된 변경이 있다 (dirty.txt). 중단"; exit 1; fi
shasum -a 256 config/env_v0.toml config/env_v2_ab.toml > "$D/config_sha256.txt"
if ! uv run --frozen --with pytest pytest tests -q > "$D/pytest.log" 2>&1; then err "테스트 실패 (pytest.log). 중단"; exit 1; fi
log "커밋 $(head -1 $D/commit.txt), 테스트 $(tail -1 $D/pytest.log)"
# 보상 검증: 후보마다 안전 순항 속도(배율 1.0)가 평균 보상 최고여야 한다 (공유 문서 "정한 것" 2절)
uv run --frozen python scripts/check_reward.py config/env_v0.toml config/env_v2_ab.toml \
  > "$D/check_reward.txt" 2>&1 || { err "check_reward.py 실패 (check_reward.txt). 중단"; exit 1; }
if [ "$(grep -c '가장 높은 배율: 1 ' "$D/check_reward.txt")" -ne 2 ]; then err "보상 검증 통과 못 함 (check_reward.txt). 중단"; exit 1; fi
log "보상 검증 통과 (2개 설정 모두 안전 속도 최적)"
# 규칙 정책 충돌 0건: 바꾼 학습 시나리오(ab)에서도 미리 감속하면 피할 수 있어야 한다 (공유 문서 "정한 것" 2절, B 조건)
rm -rf "$D/rule_check"
uv run --frozen python scripts/evaluate.py --rule mass_aware --config config/env_v2_ab.toml --eval-list 220 \
  --masses 0 2 10 25 --episodes "$RL_EP" --out-dir "$D/rule_check" > "$D/rule_check.log" 2>&1 \
  || { err "규칙 정책 평가 실패 (rule_check.log). 중단"; exit 1; }
read -r n_rule n_rule_col < <(python3 -c 'import csv, sys; r = list(csv.DictReader(open(sys.argv[1])))
print(len(r), sum(x["outcome"] == "collision" for x in r))' "$D/rule_check/episodes.csv")
if [ "$n_rule" -ne $((RL_EP * 8)) ] || [ "$n_rule_col" -ne 0 ]; then err "규칙 정책 점검 실패: ${n_rule}판 중 충돌 ${n_rule_col}건 (rule_check/). 중단"; exit 1; fi
log "규칙 정책 점검 통과 (ab 시나리오, ${n_rule}판 충돌 0건)"

config_of() {   # 후보 → 학습 설정
  case "$1" in base) echo config/env_v0.toml ;; ab) echo config/env_v2_ab.toml ;; esac
}
export -f config_of

# ---------------------------------------------------------------- 1. 학습
train_one() {   # $1 = 후보, $2 = 비교군, $3 = 시드
  if grep -q "^$1 $2 $3 " "$D/runs.txt" 2>/dev/null; then
    dir=$(grep "^$1 $2 $3 " "$D/runs.txt" | tail -1 | cut -d' ' -f4)
    [ -f "$dir/final_model.zip" ] && return 0
  fi
  logf="$D/train_$1_$2_s$3.log"
  if uv run --frozen python scripts/train_ppo.py --variant "$2" --seed "$3" --steps "$STEPS" \
       --config "$(config_of "$1")" --val-config config/env_v0.toml --tag "$1" \
       --eval-every "$EVAL_EVERY" --threads 1 > "$logf" 2>&1; then
    dir=$(grep "^저장:" "$logf" | sed -E 's/^저장: ([^ ]+).*/\1/')
    echo "$1 $2 $3 $dir" >> "$D/runs.txt"
    log "학습 $1 $2 시드 $3 완료"
  else
    err "학습 $1 $2 시드 $3 실패 ($logf)"
  fi
}
export -f train_one
log "== 1. 학습 시작 (후보 × 비교군 × 시드, 동시 $JOBS)"
for s in $SEEDS; do for c in $CANDS; do for v in $VARIANTS; do echo "$c $v $s"; done; done; done |
  xargs -P "$JOBS" -n 3 bash -c 'train_one "$0" "$1" "$2"'
log "== 1. 학습 끝 ($(wc -l < $D/runs.txt)개 기록)"

# ---------------------------------------------------------------- 2. 평가 (env_v0 시나리오)
eval_one() {   # $1 = 후보, $2 = 비교군, $3 = 시드, $4 = 실행 폴더
  out="$D/eval_$1_$2_s$3"
  [ -f "$out/episodes.csv" ] && return 0
  rm -rf "$out"
  if uv run --frozen python scripts/evaluate.py "$4" --config config/env_v0.toml --masses 0 2 10 25 \
       --episodes "$RL_EP" --mass-obs exact --out-dir "$out" > "$out.log" 2>&1; then
    log "평가 $1 $2 시드 $3 완료"
  else
    err "평가 $1 $2 시드 $3 실패 ($out.log)"
  fi
}
export -f eval_one
log "== 2. 평가 시작 (동시 $JOBS)"
sort -u "$D/runs.txt" | xargs -P "$JOBS" -n 4 bash -c 'eval_one "$0" "$1" "$2" "$3"'
n_err=$(cat "$D/errors.txt" 2>/dev/null | wc -l | tr -d " ")
log "== 모두 끝. 학습 $(sort -u $D/runs.txt | wc -l), 평가 $(ls $D/eval_*/episodes.csv 2>/dev/null | wc -l), 오류 ${n_err}건"
touch "$D/ALL_DONE"

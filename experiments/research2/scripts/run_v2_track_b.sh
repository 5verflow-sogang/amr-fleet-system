#!/bin/bash
# 연구 2 v2 트랙 B (학습 시나리오) 튜닝. 다시 실행하면 끝난 것은 건너뛰고 이어서 한다.
#   caffeinate -i scripts/run_v2_track_b.sh
#
# 후보: base(v1 설정 env_v0) · b1(popout_prob 0.9) · b2(popout_gap [0.5, 1.0]) × 비교군 proposed·r2 × 시드 210~219.
# 검증(체크포인트 선택·eval_common)과 평가는 모두 env_v0 시나리오다 (--val-config, evaluate.py --config 기본값).
# 평가: 학습 질량 0·2·10·25 kg × S1·S2 × 200회, 질량 정보 정확. 튜닝 평가 목록 = 학습 시드.
# 채택 규칙은 실행 전에 정한 공유 문서 "정한 것 (10/17 확정)" 3절을 따른다. 이 스크립트는 판정하지 않는다.
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/v2_track_b}
SEEDS=${SEEDS:-$(seq -s ' ' 210 219)}
CANDS=${CANDS:-base b1 b2}
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
shasum -a 256 config/env_v0.toml config/env_v2_b1.toml config/env_v2_b2.toml config/env_v3_info.toml > "$D/config_sha256.txt"
if ! uv run --frozen --with pytest pytest tests -q > "$D/pytest.log" 2>&1; then err "테스트 실패 (pytest.log). 중단"; exit 1; fi
log "커밋 $(head -1 $D/commit.txt), 테스트 $(tail -1 $D/pytest.log)"

config_of() {   # 후보 → 학습 설정
  case "$1" in base) echo config/env_v0.toml ;; b1) echo config/env_v2_b1.toml ;; b2) echo config/env_v2_b2.toml ;; info) echo config/env_v3_info.toml ;; esac
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

#!/bin/bash
# 연구 2 v3 벌점 곡선 실험: 충돌 벌점 단계(−100·−300·−1000)마다 제안·R2를 학습해, 같은 충돌률에서의 도착 시간을 비교한다.
# 다시 실행하면 끝난 것은 건너뛰고 이어서 한다.
#   caffeinate -i scripts/run_v3_frontier.sh
#
# 질문: 같은 안전(충돌률)에서 질량을 아는 제안 방법이 가벼울 때 더 빨리 가는가 (R2는 질량을 모른다).
# 벌점 크기를 바꾼 정책들은 안전–속도 곡선(파레토) 위의 점이 된다. 제약 RL(축소 실험 1·2차 실패, run_v3_pilot.sh)은
# 그 곡선의 한 점을 자동으로 찾는 장치일 뿐이라, 곡선을 직접 그려 비교한다.
# 설정: config/env_v3_pen{100,300,1000}.toml = env_v0(G)에서 충돌 고정 벌점과 보상 자르기 한도(10 → 100)만 다르다.
#   자르기 한도 10에서는 정규화한 충돌 벌점이 −10에서 잘려 단계 차이가 사라진다 (v2 트랙 A: −140·−240·−340 모두 −10)
# 후보 pen100·pen300·pen1000 × 비교군 proposed·r2 × 시드 310~329 (20개), 300만 스텝. 검증·평가는 env_v0 시나리오.
# 평가: 학습 질량 0·2·10·25 kg × S1·S2 × 200회, 질량 정보 정확 (v1·v2와 같다).
#
# 판정 기준 (결과 보기 전에 정함. scripts/judge_v3_frontier.py가 계산한다):
#   F1 핵심: 같은 충돌률(학습 질량 전체, R2 곡선 선형 보간)에서 0 kg 도착 시간 제안 − R2의 95% CI 상한 < 0
#   F2 질량 의존: (25 kg 시간 − 0 kg 시간)의 제안 − R2 (벌점 단계 평균)의 95% CI 하한 > 0
#   F3 보고: 무너진 시드(학습 질량 성공 < 85% 또는 시간 초과 > 10%) 수 (빼지 않는다)
#   F1의 정의(전체 충돌률로 맞추고 0 kg 시간을 본다)는 v2 트랙 A 미리보기(시드 200~209)를 본 뒤 정했다.
#   그 데이터에서는 F1 −0.79 s (−1.53 ~ +0.22), F2 +1.43 s (+1.16 ~ +1.67)였다. 판정은 새 시드 310~329로만 한다.
# 학습 전 점검: 단계마다 check_reward.py(안전 속도가 최적)와 check_reward_traps.py(지나치기·기어가기·안 가기가 정상 주행보다
# 낮음) 통과. 못 넘으면 시작하지 않는다.
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/v3_frontier}
SEEDS=${SEEDS:-$(seq -s ' ' 310 329)}
CANDS=${CANDS:-pen100 pen300 pen1000}
VARIANTS=${VARIANTS:-proposed r2}
STEPS=${STEPS:-3000000}
EVAL_EVERY=${EVAL_EVERY:-100000}
RL_EP=${RL_EP:-200}
TRAP_EP=${TRAP_EP:-50}
JOBS=${JOBS:-6}
mkdir -p "$D"
P="$D/progress.txt"
export D P STEPS EVAL_EVERY RL_EP
log() { echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$P"; }
err() { echo "$(date '+%m-%d %H:%M:%S') $*" >> "$D/errors.txt"; log "오류: $*"; }
export -f log err   # xargs 안의 bash에서 macOS /usr/bin/log 대신 이 함수를 쓰게
log "설정: D=$D SEEDS=[$SEEDS] CANDS=[$CANDS] VARIANTS=[$VARIANTS] STEPS=$STEPS RL_EP=$RL_EP JOBS=$JOBS"

config_of() { echo "config/env_v3_$1.toml"; }   # 후보 → 학습 설정
export -f config_of

# ---------------------------------------------------------------- 0. 사전 점검 (실패하면 시작하지 않는다)
log "== 0. 사전 점검"
git rev-parse HEAD > "$D/commit.txt"
git rev-parse --abbrev-ref HEAD >> "$D/commit.txt"
git status --porcelain -- . ':!raw' > "$D/dirty.txt"
if [ -s "$D/dirty.txt" ]; then err "커밋 안 된 변경이 있다 (dirty.txt). 중단"; exit 1; fi
cfgs=$(for c in $CANDS; do config_of "$c"; done)
shasum -a 256 config/env_v0.toml $cfgs > "$D/config_sha256.txt"
if ! uv run --frozen --with pytest pytest tests -q > "$D/pytest.log" 2>&1; then err "테스트 실패 (pytest.log). 중단"; exit 1; fi
log "커밋 $(head -1 $D/commit.txt), 테스트 $(tail -1 $D/pytest.log)"
uv run --frozen python scripts/check_reward.py $cfgs > "$D/check_reward.txt" 2>&1 \
  || { err "check_reward.py 실패 (check_reward.txt). 중단"; exit 1; }
n_cfg=$(echo $cfgs | wc -w | tr -d ' ')
if [ "$(grep -c '가장 높은 배율: 1 ' "$D/check_reward.txt")" -ne "$n_cfg" ]; then err "보상 검증 통과 못 함 (check_reward.txt). 중단"; exit 1; fi
uv run --frozen python scripts/check_reward_traps.py $cfgs --lists 310 311 --episodes "$TRAP_EP" > "$D/check_traps.txt" 2>&1 \
  || { err "check_reward_traps.py 실패 (check_traps.txt). 중단"; exit 1; }
# rush·creep·halt의 "stop 대비"가 모두 음수여야 한다 (정상 주행보다 싼 실패가 없음)
bad=$(awk '$1 ~ /^(rush|creep|halt)$/ { for (i = 1; i <= NF; i++) if ($i == "대비") { v = $(i + 1); if (v + 0 >= 0) print FILENAME": "$0 } }' "$D/check_traps.txt")
if [ -n "$bad" ]; then err "함정 점검 통과 못 함: $bad. 중단"; exit 1; fi
log "보상 검증·함정 점검 통과 (설정 ${n_cfg}개)"

# ---------------------------------------------------------------- 1. 학습
train_one() {   # $1 = 후보, $2 = 비교군, $3 = 시드
  if grep -q "^$1 $2 $3 " "$D/runs.txt" 2>/dev/null; then
    dir=$(grep "^$1 $2 $3 " "$D/runs.txt" | tail -1 | cut -d' ' -f4)
    [ -f "$dir/final_model.zip" ] && return 0
  fi
  logf="$D/train_$1_$2_s$3.log"
  if uv run --frozen python scripts/train_ppo.py --variant "$2" --seed "$3" --steps "$STEPS" \
       --config "$(config_of "$1")" --tag "$1" --eval-every "$EVAL_EVERY" --threads 1 > "$logf" 2>&1; then
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

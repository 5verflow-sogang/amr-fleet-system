#!/bin/bash
# 연구 2 v3 정보 실험: 같은 제안 정책에 질량 정보만 바꿔 준다 (학습 없음). 확인 실험의 정책(raw/v3_confirm, 벌금 −100,
# 시드 330~369)을 그대로 쓴다. 다시 실행하면 끝난 평가는 건너뛴다.
#   scripts/run_v3_info.sh
#   uv run python scripts/judge_v3_info.py raw/v3_info
#
# 질문: 질량 정보가 의미 있는가. 따로 학습한 정책끼리(제안 vs R2) 비교하면 정책 사이 편차가 효과를 덮었다. 같은 정책에
# 정보만 바꿔 주면 그 편차가 빠진다 (제안 정책은 학습의 20%를 질량 가림 상태로 연습했다).
# ① 정보 있음 vs 없음 (exact vs unknown) ② 틀린 정보 (told0: 늘 "0 kg", told25: 늘 "25 kg") ③ 반응 곡선 (질량 칸만 바꿨을 때 속도 명령)
# 평가: env_v0 시나리오, 학습 질량 0·2·10·25 kg × S1·S2 × 200회, 평가 목록 = 학습 시드 (exact는 확인 실험 평가와 같은 판이다)
# 판정 기준 (결과 보기 전에 정함. scripts/judge_v3_info.py 설명과 같다):
#   I1 효율: 0 kg 도착 시간 exact − unknown (같은 정책·시나리오, 둘 다 성공한 판) 95% CI 상한 < 0
#   I2 안전 유지: 학습 질량 전체 충돌률 exact − unknown 95% CI 상한 ≤ +2%p (가벼울 때 빨라진 만큼 충돌이 늘지 않는지까지 보려고 전체로)
#   I3 인과: 25 kg 충돌률 told0 − exact 95% CI 하한 > 0
#   셋 다 통과하면 "질량 정보가 정책 안에서 효율을 높이고, 정책이 그 정보에 의존한다"로 쓴다
#   타당성 점검 I0: 0 kg 도착 시간 제안 unknown − R2의 CI 하한 > 0이면 I1을 "덜 연습한 영향이 섞인 상한"으로 쓴다 (판정 아님)
cd "$(dirname "$0")/.." || exit 1          # experiments/research2
D=${D:-raw/v3_info}
RUNS=${RUNS:-raw/v3_confirm/runs.txt}
RL_EP=${RL_EP:-200}
JOBS=${JOBS:-6}
mkdir -p "$D"
P="$D/progress.txt"
export D P RL_EP
log() { echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$P"; }
export -f log
log "설정: D=$D RUNS=$RUNS RL_EP=$RL_EP JOBS=$JOBS"
git rev-parse HEAD > "$D/commit.txt"
git status --porcelain -- . ':!raw' > "$D/dirty.txt"
if [ -s "$D/dirty.txt" ]; then log "커밋 안 된 변경이 있다 (dirty.txt). 중단"; exit 1; fi
eval_one() {   # $1 = 비교군, $2 = 시드, $3 = 실행 폴더
  out="$D/eval_$1_s$2"
  [ -f "$out/episodes.csv" ] && return 0
  rm -rf "$out"
  if uv run --frozen python scripts/evaluate.py "$3" --config config/env_v0.toml --masses 0 2 10 25 \
       --episodes "$RL_EP" --mass-obs exact unknown told0 told25 --out-dir "$out" > "$out.log" 2>&1; then
    log "평가 $1 시드 $2 완료"
  else
    log "오류: 평가 $1 시드 $2 실패 ($out.log)"
  fi
}
export -f eval_one
log "== 1. 평가 (R2는 질량을 보지 않아 exact 하나만 돈다)"
sort -u "$RUNS" | awk '{print $2, $3, $4}' | xargs -P "$JOBS" -n 3 bash -c 'eval_one "$0" "$1" "$2"'
log "== 2. 반응 곡선"
uv run --frozen python scripts/probe_v3_mass_response.py "$RUNS" --out "$D" > "$D/probe.log" 2>&1 \
  && log "반응 곡선 완료" || log "오류: 반응 곡선 실패 (probe.log)"
log "== 모두 끝. 평가 $(ls $D/eval_*/episodes.csv 2>/dev/null | wc -l | tr -d ' ')개"
touch "$D/ALL_DONE"

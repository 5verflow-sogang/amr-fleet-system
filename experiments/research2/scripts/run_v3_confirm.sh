#!/bin/bash
# 연구 2 v3 효율 확인 실험: 벌점 곡선 실험의 탐색 결과("벌금 −100에서 충돌률은 같고 0 kg에서 제안이 0.81 s 빠름")를
# 새 시드에서 미리 정한 기준으로 다시 판정한다. 실행은 run_v3_frontier.sh를 그대로 쓰고 후보·시드·폴더만 바꾼다.
#   caffeinate -i scripts/run_v3_confirm.sh
#   uv run python scripts/judge_v3_confirm.py raw/v3_confirm --cand pen100
#
# 후보 pen100 (config/env_v3_pen100.toml) × 비교군 proposed·r2 × 시드 330~369 (40개, 벌점 곡선 실험과 겹치지 않음), 300만 스텝.
# 시드 수: 벌점 곡선 실험(시드 20개)에서 충돌률 차이의 95% CI 폭이 약 ±2.1%p라, 20개로는 차이가 0이어도 C2(+2%p)를 넘기 어렵다.
#   결과를 보기 전에 C2 기준은 그대로 두고 시드를 40개로 늘렸다 (폭 약 ±1.5%p 예상). 기준을 느슨하게 하는 대신 정밀도를 높인 것이다
# 판정 기준 (결과 보기 전에 정함. scripts/judge_v3_confirm.py 설명과 같다):
#   C1 효율: 0 kg 성공 판 평균 도착 시간의 제안 − R2 (시드별 차이의 평균) 95% CI 상한 < 0
#   C2 안전 비열등: 충돌률(돌발 장애물이 나타난 판 분모)의 제안 − R2 95% CI 상한 ≤ +2%p
#   C1·C2 모두 통과하면 "같은 안전에서 가벼울 때 더 빠름"을 확인된 결과로 쓴다. 하나라도 미달이면 탐색 결과로만 쓴다
#   보조 C3: (25 kg − 0 kg 시간)의 제안 − R2 95% CI 하한 > 0 (질량 의존의 세 번째 확인)
cd "$(dirname "$0")/.." || exit 1
D=${D:-raw/v3_confirm} SEEDS=${SEEDS:-$(seq -s ' ' 330 369)} CANDS=${CANDS:-pen100} exec scripts/run_v3_frontier.sh

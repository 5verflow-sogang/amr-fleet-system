# experiments

연구 실험의 시나리오 설정, 실행 스크립트, 분석 노트북을 둔다.

- `research1/`: 중앙집중 Fleet 통신 지연 내성과 분산 전환 임계점 (하연)
- `cte_baseline/`: 경로 추종 CTE 1회 측정 (Gazebo 월드 v0, 고정 경로, Nav2 없음) (이진)
- `research2/`: 연구 2 (AI/ML, 연우). 주제는 10/5 회의에서 확정한다([명세서 4장](../docs/project-spec.md))
- `payload_braking/`: 적재 질량 제동 거리 (이진)
- `odom_drift/`: 휠 오도메트리 드리프트, 10 m 사각형 복귀 오차 (지수, D9)
- `bt_cycle/`: Fleet 작업 → BT → Nav2 이동 → ArUco 도킹 → COMPLETED 한 사이클과 단일 작업 성공률 (지수, 4주차 작업 1·3)

원자료(rosbag, trace)는 커밋하지 않는다. `raw/` 아래에 두면 git에서 무시되며, 공유 드라이브 경로를 각 실험 README에 기록한다.

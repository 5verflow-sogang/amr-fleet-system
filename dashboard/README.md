# dashboard

관제 대시보드 + 직원 호출. 시연 영상에서 관제가 무엇을 판단했고 그래서 무엇이 나아졌는지를 대시보드만 봐도 알 수 있게 한다.
화면·문구·동작은 「관제 대시보드 개발 명세서」를 따른다.

- ROS 패키지가 아니다. `COLCON_IGNORE`로 colcon 빌드에서 제외된다.
- `index.html` 한 장이다. 외부 라이브러리 없이 `rosbridge_server`(WebSocket) 프로토콜을 직접 쓴다. 대시보드 코드가 ROS 노드를 직접 import하지 않는다.
- 필요한 토픽은 [docs/api.md](../docs/api.md)의 인터페이스만 사용한다. 대시보드 전용 데이터가 필요하면 `amr_msgs`에 메시지를 추가하는 절차를 따른다.
- 시계열 KPI는 Prometheus/Grafana(`docker/grafana/`)가 담당하므로, 이 대시보드는 지도 위 로봇 위치·상태·작업·알림처럼 Grafana로 표현하기 어려운 화면에 집중한다.

## 실행

```bash
docker compose build                                   # rosbridge_server가 들어간 이미지 (amr_bringup exec_depend)
docker compose up -d fleet
FAKE_ROBOTS_CONFIG=fake_robots_restaurant.yaml docker compose --profile fake up -d fake-robots   # 식당 위치 모의 로봇 5대
docker compose --profile dashboard up -d
# 브라우저: http://localhost:8080
```

`dashboard` 서비스는 host 네트워크에서 화면(8081)과 rosbridge(9091)를 열고, `dashboard-proxy`가 호스트의 8080·9090으로
중계한다(macOS·Windows Docker Desktop은 host 네트워크 포트가 호스트에 열리지 않는다. `gui`·`gui-proxy`와 같은 방식).
화면은 1280×720 고정이다. 녹화는 브라우저 창을 1280×720으로 두고 2배로 받는다.

## 화면

| 영역 | 내용 |
|---|---|
| 상단 바 | 시각, 연결 상태 |
| 숫자 칸 | 홀 투입(최근 3분 그래프, 최소 대수), 최장 대기(배정 전), 안전 정지, 완료(서빙·회수). 셈은 페이지를 연 시각부터 |
| 지도 | 로봇 위치·상태·가는 방향, 붐비는 구역, 경로(없으면 목적지까지 점선), 배정 후보 고리, 고른 테이블 |
| 관제 판단 | 방금 내린 판단 1건과 이유 (배정, 회수 먼저, 요청 순서대로, 서빙 먼저, 충전 보냄) |
| 작업 순서 | 로봇을 기다리는 작업(처리 순서대로)과 방금 배정된 작업(5초). 대기 막대는 1분에 꽉 찬다 |
| 로봇 | 5대의 상태·가는 곳·배터리(5칸, 한 칸 20%), 도움이 필요한 로봇 수 |
| 호출 | 테이블을 골라 빈 그릇 회수, 음식 서빙(누르면 매장 5개가 펼쳐진다) |

로봇·작업 순서 줄·관제 판단을 누르면 로봇 창·작업 창이, 홀 투입 칸을 누르면 5대 배터리 창이 뜬다.
색은 평소 회색, 관제가 방금 판단한 것과 고른 것 파랑, 충전 주황, 문제만 빨강이다. 작업은 어디서나 같은 이름표(서빙·회수·충전)로 보인다.

## 주소 옵션

| 옵션 | 뜻 | 기본값 | 쓰는 곳 |
|---|---|---|---|
| `ws` | rosbridge 주소. `off`면 연결하지 않는다(시험) | `ws://<페이지 호스트>:9090` | 공통 |
| `collect_priority` | 회수 `priority` | 200 | 2 1막 끔: 100 ("요청 순서대로") |
| `collect_type` | 회수 `task_type`. #302(`TYPE_COLLECT`)가 dev에 들어오면 3 | 0 | #302 병합 뒤 |
| `aging_rate` | 작업 순서 계산. 서버 `assignment.aging_rate`와 같게 | 0 | 2 2막 켬: 1.667 ("서빙 먼저") |
| `clock` | 시뮬레이션 0초에 보일 시각 (`12:07:00`). 숫자 칸의 "12:07부터"도 이 시각 | 없음 | 장면마다 스토리 시각 |
| `min_hall` | 홀 투입 최소 대수 (숫자 옆 "최소 3대", 그래프에서 그 아래는 옅은 띠·빨간 선, 모자라면 숫자 빨강) | 없음 | 3: 3 |

## 받는 토픽

| 토픽 | 쓰는 곳 |
|---|---|
| `/fleet/robot_states` | 지도, 로봇, 숫자 칸, 배터리 창 |
| `/fleet/task_table` | 작업 순서, 관제 판단, 호출 중복 막기 |
| `/fleet/task_status` | 로봇 칩 단계, 완료(서빙·회수), 작업 창 시각 |
| `/fleet/congestion` | 붐비는 구역 |
| `/amr_0N/safety/status` | 안전 정지 |
| `/clock` | 시각 (없으면 브라우저 시각. 처음 오면 시간 기준을 시뮬레이션 시간으로 바꾼다) |
| `/fleet/assignment_decisions` | 배정 이유와 후보 비교 (메시지 `amr_msgs/AssignmentDecision`이 생기면 바로 쓴다. 없으면 큰 글씨와 작업 줄만) |
| `/amr_0N/plan` | 경로 (없으면 목적지까지 점선) |

보내는 토픽은 `/fleet/task_requests`(`amr_msgs/Task`, api.md 4-6절) 하나다. `task_id`는 `collect_`·`delivery_` + UUID다.

## 시험

```bash
pip install playwright && playwright install chromium
python3 dashboard/test/check_ui.py --shots dashboard/test/out/shots   # ROS 없이 명세 3장 완료 기준 + 5장 문구 규칙, 시연 장면 13장
python3 dashboard/test/check_stack.py --url http://localhost:8080 \
  --restart "docker compose --profile dashboard restart dashboard"   # 실제 스택 (위 실행 뒤). 호출 → 배정, 다시 연결
```

`check_ui.py`는 `window.dashTest.feed(topic, msg)`로 메시지를 넣고 Playwright 가짜 시계로 시간을 돌린다(1분 넘은 대기, 30초 넘은
안전 정지 등을 실제로 기다리지 않고 본다). `check_stack.py`는 모의 로봇(`fake_robot_node`: 시뮬레이터 없이 위치만 계산하는 가짜 로봇)으로
호출부터 완료까지 확인한다. 모의 로봇은 배터리 100% 고정이고 안전 상태를 보내지 않으므로 배터리·안전 정지는 `check_ui.py`가 맡는다.

# 장면 3 실제 Gazebo 녹화

`demo_s3_gazebo`는 실제 Gazebo·Nav2·amr_bt·Fleet 서버로 충전 스케줄링을 비교한다. 가짜 로봇 미리보기 `demo_s3_fake`와 별개다.

## 같은 시작 조건

Nav2와 카메라·측정 bag이 준비된 뒤 `demo_bt_group.launch.py`로 작업 노드 5개를 함께 시작한다. 로봇을 순차 스폰하는 동안 배터리가 먼저 소모되지 않도록 하기 위한 장면 전용 기동 방식이다. 다른 장면은 기존처럼 작업 노드를 로봇과 함께 시작한다.

두 조건의 시작 배터리는 31·32·85·70·40%다. 가상 부하로 초당 0.2%가 소모되고, 충전 중에는 0.4 %/s로 60%까지 충전한다. 이동 거리 소모는 0이다. 경로·대기 차이 때문에 비교 조건의 배터리 소모가 달라지지 않도록 한 영상용 압축 모델이다. 배터리나 모드를 녹화 중 외부에서 덮어쓰지 않는다.

- 끔: 20% 미만이면 자동 충전, 최소 운영 대수 제한 없음.
- 켬: 30% 미만이면 선제 충전, 홀 최소 3대 유지(충전 이동 포함 최대 2대).

## 동선과 작업

낮은 배터리 로봇은 충전소 북쪽에 서고, 가장 먼저 충전을 마치는 로봇이 오른쪽 패드부터 사용한다. 복귀할 때 아직 충전 중인 다른 로봇 앞을 가로지르지 않게 했다.

북쪽 서빙 요청은 30초마다 두 매장을 번갈아 사용한다. 남쪽 회수는 75·105초에 같은 테이블에서 요청한다. 회수는 퇴식대 옆면 `return_s3_side`에서 하역해 충전 동선과 분리한다. 요청 시각·작업·시작 위치·소모·충전 속도는 켬/끔이 같고, 로봇 배정은 실제 Fleet 알고리즘이 결정한다.

`stations_restaurant_s3.yaml`은 서버와 장면 스크립트가 공유한다. 가구를 이동시키지 않고 서비스 정차 자세만 바꾼다. 식당 정답 위치추정과 서쪽 손님 7명을 쓰고, 추적 노드는 생략하되 LiDAR 비용 지도와 안전 정지는 유지한다.

## 실행

로컬 컨테이너는 순차 실행한다. 같은 호스트에서 병렬 실행할 때는 ROS 도메인과 Gazebo partition을 모두 분리해야 한다. 하네스의 기본 도메인은 각 컨테이너 첫 시도에서 1이므로 컨테이너 분리만으로 통신이 격리되지는 않는다.

Actions의 Fleet Regression Test에서 같은 코드 SHA로 실행한다.

| 입력 | 켬 | 끔 |
| --- | --- | --- |
| scenario | `demo_s3_gazebo` | `demo_s3_gazebo` |
| repeats | `1` | `1` |
| params | `policy=on slot=c1 duration_s=180` | `policy=off slot=c2 duration_s=180` |
| delay_ms / loss_pct | `0 / 0` | `0 / 0` |

결과 `video/top.mp4`, `video/south.mp4`, `video/video.json`, `dashboard.jsonl`, `rosbag/`, `scene.json`을 확인한다. 카메라와 대시보드는 `/demo/t0` 및 `video/video.json`의 첫 프레임 시각으로 맞춘다.

```bash
python3 tools/demo/record_dashboard.py <run>/dashboard.jsonl --out overview.mp4 --query 'min_hall=3'
python3 tools/demo/record_dashboard.py <run>/dashboard.jsonl --out battery.mp4 --view battery --query 'min_hall=3'
# 끔의 query: 'min_hall=3&charge_why=배터리 20% 미만'
```

## 녹화 성공 기준

Actions 완료 표시만으로 성공으로 보지 않는다.

1. 끔: 실제 `MODE_CHARGING` 3대가 겹치는 구간과 남쪽 충전소 주차 모습을 확인한다.
2. 켬: 충전 작업(이동 포함) 최대 2대, 비충전 로봇 최소 3대와 실제 업무 수행을 확인한다.
3. 켬: 충전 완료 후 나오는 로봇과 다음 충전 로봇을 확인한다.
4. 배터리 상승, 작업표, 대시보드, Gazebo 위치가 일치해야 한다.

충전 작업 기준·충전 모드 기준·새 작업을 받을 수 있는 배터리 기준 대수는 구분한다. 주문 대기가 켬에서 더 짧다는 주장은 실측이 뒷받침할 때만 사용한다.

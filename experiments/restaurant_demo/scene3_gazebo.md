# 장면 3 실제 Gazebo 녹화

`demo_s3_gazebo`는 기존 `demo_s3`의 충전 정책과 녹화 파이프라인을 사용하되 서비스 정차 자세·출발 위치·주문 순서를 Gazebo 주행에 맞춘다. 실제 Gazebo 물리, Nav2, amr_bt 배터리, Fleet 배정 서버로 동작한다. 가짜 로봇용 `demo_s3_fake`와 별개다.

## 동일 조건과 비교 조건

두 조건 모두 로봇 5대, 시작 배터리 34·33·85·70·39.5%, 이동 0.6 %/m, 정지 소모 0, 충전 0.6 %/s, 충전 목표 60%다. 같은 장면 파일과 seed를 사용한다.

- 끔: 자동 충전은 동작하며, 20% 미만에서 충전한다. 최소 운영 대수 제한은 없다(충전기는 3개).
- 켬: 30% 미만에서 선제 충전하며, 최소 3대를 남긴다(충전 이동 포함 최대 2대).

식당 정답 위치추정과 서쪽에 배치한 손님 7명을 사용한다. 추적 노드는 생략하지만 LiDAR 전역·지역 비용 지도와 안전 정지는 유지한다. 가상 배터리를 시연 시간에 맞춰 압축한 설정이며 실제 하드웨어의 운용 시간 성능을 뜻하지 않는다.

## 동선

처음에는 북쪽 매장 2곳의 서빙과 남쪽 원형 테이블 회수로 나눈다. 이후 배정은 실제 Fleet의 최소 거리 배정이 결정한다. 원형 테이블은 남쪽 면에서 회수하고 서쪽 로봇부터 출발시킨다. 대기 중인 5호는 남동쪽 공터에 둔다.

`stations_restaurant_s3.yaml`은 가구를 이동시키지 않고 서비스 정차 자세만 바꾼다. 서버와 장면 스크립트가 이 파일을 공유한다. 로봇이 카운터·테이블 바로 앞에서 방향 전환하다 안전 노드에 막히는 문제를 줄이기 위한 설정이다.

## 실행

Actions의 Fleet Regression Test에서 같은 코드 SHA로 실행한다.

| 입력 | 켬 | 끔 |
| --- | --- | --- |
| scenario | `demo_s3_gazebo` | `demo_s3_gazebo` |
| repeats | `1` | `1` |
| params | `policy=on slot=c1 duration_s=300` | `policy=off slot=c2 duration_s=300` |
| delay_ms / loss_pct | `0 / 0` | `0 / 0` |

결과 `video/top.mp4`, `video/south.mp4`, `dashboard.jsonl`, `rosbag/`, `scene.json`을 확인한다. 카메라 영상과 대시보드는 `/demo/t0` 및 `video.json`의 첫 프레임 시각으로 맞춘다.

```bash
python3 tools/demo/record_dashboard.py <run>/dashboard.jsonl --out overview.mp4 --query 'min_hall=3'
python3 tools/demo/record_dashboard.py <run>/dashboard.jsonl --out battery.mp4 --view battery --query 'min_hall=3'
# 끔에는 위 명령의 query를 'min_hall=3&charge_why=배터리 20% 미만'으로 준다.
```

## 녹화 성공 기준

Actions 완료 표시만으로 성공으로 보지 않는다.

1. 끔: 실제 `MODE_CHARGING` 로봇 3대가 겹치는 구간과 남쪽 충전소에 주차한 모습을 확인한다.
2. 켬: 충전 작업을 맡은 로봇(이동 포함)이 2대를 넘지 않고, 남은 로봇이 실제 업무를 수행한다.
3. 켬: 충전 완료 후 빠져나오는 로봇과 다음 충전 로봇을 확인한다.
4. 배터리 상승, 작업표, 대시보드 충전 표시와 Gazebo 주차 위치가 일치해야 한다.

홀 투입 대수(충전 이동 포함 제외), 실제 충전 모드 기준 대수, 배터리 20% 이상인 비충전 로봇 수를 구분해서 본다. 주문 대기가 켬에서 더 짧다는 주장은 측정 결과가 뒷받침할 때만 사용한다.

# tools/demo: 식당 시연 끔·켬 비교 영상

하네스 녹화 두 개(끔·켬)를 「식당 시연 영상 편집 가이드」 화면 배치(1600×900)로 붙이고, 녹화 기록으로 강조 표시를 그린다.
시청자가 어느 로봇을 보면 되는지, 주문이 언제 들어왔는지를 화면에서 바로 알게 하려는 것이다.

| 표시 (`layers`) | 무엇 | 기록 |
| --- | --- | --- |
| `request` | 주문 자리에 주황 고리가 퍼지고 "주문 · 배식대 1 → 1번 테이블" 배지 | `/demo/t0` (장면 스크립트) |
| `assign` | 주문 자리 → 맡은 로봇 점선, 그 로봇 동그라미(끔 빨강 · 켬 파랑) + "1호 배정" | `/fleet/task_table` `assignee_id` |
| `hero` · `dim` | 맡은 로봇을 따라다니는 동그라미, 그 밖은 어둡게 | `/fleet/robot_states` |
| `candidates` | 후보 로봇(1-1: 1호·3호)에 흰 동그라미와 주문 자리까지 직선 거리 | `/fleet/robot_states` |
| `zones` | 혼잡 구역 반투명 빨강 + "혼잡 1.0" | `/fleet/congestion` |
| `card` · `clock` · `arrived` | 작업 알림 카드, "요청 후 12.3 s", "도착 · 요청 후 23.4 s" | `/fleet/task_status` 단계 |

월드 좌표 → 화면은 `amr_gazebo/config/demo_cameras_restaurant.yaml`의 카메라 자세·화각으로 옮긴다 (핀홀). 탑뷰는 물론
눈높이 카메라(`west`)에도 그린다. 맨 끝 컷은 결과 장표(요청 → 도착 시간 막대, 멈춘 횟수, 지표 정의)다.

## 쓰는 법

실행 폴더는 버킷 그대로 받는다: `gs://amr-fleet-results/runs/demo_s11-<on|off>/<날짜_실행>/<실행 id>/` 아래 `video/`와 `rosbag/`.

```bash
# 컨테이너 (rosbag2_py, amr_msgs, ffmpeg). 가이드 글꼴이 없으면 한 번 설치한다
apt-get install -y fonts-wqy-zenhei python3-pil
source /ros2_ws/install/setup.bash
python3 tools/demo/compose.py extract runs/off           # → runs/off/timeline.json (핵심 작업·배정·도착·멈춤 출력)
python3 tools/demo/compose.py extract runs/on
python3 tools/demo/compose.py plan   --off runs/off --on runs/on                  # 컷 시각·문구·경고 (ffmpeg 없이)
python3 tools/demo/compose.py render --off runs/off --on runs/on --stills stills  # 컷마다 한 장 PNG
python3 tools/demo/compose.py render --off runs/off --on runs/on --out s11.mp4   # 완성본
```

`extract`만 ROS가 필요하다. `timeline.json`이 생기면 `plan`·`render`는 Python + Pillow(+ ffmpeg)만 있으면 어디서든 돈다
(Windows는 맑은 고딕으로 대신 그린다, `--font`로 바꾼다).

## 컷시트 (`cuts/s11.yaml`)

컷마다 카메라, 시작 시각, 완성본 길이, 배속, 멈춤, 표시, 제목·자막을 적는다. 시작 시각은 녹화마다 달라 사건으로 적는다:
`request`(0초), `assign`, `arrive`(둘 다 지난 늦은 쪽), `assign_off`·`arrive_on`처럼 한쪽, `end` ± 초. 두 실행은 각자의
`/demo/t0`를 0초로 맞춰 같은 상대 시각을 자른다 (가이드).

- 제목·자막의 `{robot_off}`·`{robot_on}`(1호·3호), `{arrive_off}`·`{arrive_on}`(초), `{stops_off}`, `{dist_amr_01}`(m)은
  녹화 기록으로 채운다. 가이드: 로봇 번호·시각·수치는 녹화 기록을 따른다
- `check: arrive_on < arrive_off`처럼 문구가 맞으려면 성립해야 하는 순서를 적으면, 녹화가 다를 때 `plan`·`render`가 경고한다
- 사건이 녹화에 없는 컷(예: 끔 로봇이 끝내 도착하지 못함)은 빼고 경고한다

완성본: 1600×900, 20 fps, H.264 CRF 17, yuv420p, 무음. 공유 전 가이드 6단계(한글 표시, 좌우 시간·배속, 자막 잘림, 숫자와
화면 일치)를 확인한다.

## 시험

```bash
python -m pytest -q tools/demo/tests     # 가짜 녹화로 투영·사건·컷시트·그리기 (ROS·ffmpeg 없이)
```

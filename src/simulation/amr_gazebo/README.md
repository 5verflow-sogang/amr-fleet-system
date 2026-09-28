# amr_gazebo

- 담당: 지수
- 빌드 타입: ament_cmake (자원 전용, C++ 코드 없음)
- 책임: 60x40m 물류센터 월드, 동적 장애물, 센서 노이즈 모델

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬레이션 시간. 다른 노드는 `use_sim_time:=true`로 쓴다 |
| Subscribe | | | |
| Service | | | |
| Action | | | |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 월드 레이아웃 (`worlds/warehouse.sdf`)

좌표계: 월드 원점 = 창고 중심, x −30~30 m, y −20~20 m, +y가 북쪽. 벽 안쪽 면 기준 60 × 40 m.
레이아웃은 [원 명세서 8장 결과 예시](../../../docs/project-spec-original.md)를 따른다.

| 구역 | 모델 이름 | 위치 (x, y) | 비고 |
| --- | --- | --- | --- |
| 입고 도크 | `dock_1`, `dock_2` | 서쪽 벽 (−30, 16), (−30, 11) | 플랫폼 1.5 m + 정차 구역(노란 표시) 3 m, +x 방향 |
| 출고 도크 | `dock_a`, `dock_b` | 동쪽 벽 (30, 16), (30, 11) | −x 방향 |
| 메인 통로 | 바닥 표시 | y 8.5 ~ 18.5 | 도크 사이 동서 통로 |
| 선반 A열 | `rack_a1`~`rack_a7` | y = 5, x = −22.5 ~ 22.5 (7.5 m 간격) | 랙 5.0 × 1.2 × 2.5 m |
| 선반 B열 | `rack_b1`~`rack_b7` | y = −2 | `rack_b4`만 x = −1.9 |
| 선반 C열 | `rack_c1`~`rack_c7` | y = −9 | |
| 열 사이 통로 | | A–B: y −1.4~4.4, B–C: y −8.4~−2.6 | 폭 5.8 m |
| 교차 통로 | | 같은 열 랙 사이 | 폭 2.5 m |
| **좁은 통로** | `rack_b3`–`rack_b4` 사이 | x −5.0 ~ −4.4, y −2.6 ~ −1.4 | **폭 0.6 m** (로봇 폭 0.4 m + 0.2 m, Nav2 KPI용) |
| 기둥 | `pillar_1`~`pillar_6` | (±11.25, 7.0), (±15, 1.5), (±7.5, −5.5) | 0.6 × 0.6 × 6 m, 통로 가운데 |
| 충전 스테이션 | `charger_c1`~`c3` | 남쪽 벽 (−24, −20), (−21, −20), (−18, −20) | 본체 0.3 m + 도킹 패드(초록 표시), +y 방향 |
| 충전 구역 | 바닥 표시 (파랑) | x −26~−16, y −20~−16 | |
| 대기 구역 | 바닥 표시 (회색) | x 16~26, y −20~−14 | |

재사용 모델은 `models/`에 있고 월드에서 `model://<이름>`으로 불러온다.
경로는 `package.xml`의 `gazebo_model_path` export를 통해 `ros_gz_sim`이 자동으로 등록한다.

아직 없는 것 (2주차 이후): 동적 장애물, 바닥 마찰·조명 튜닝, 센서 시스템 플러그인(Sensors, IMU).

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

launch 인자:

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `world` | `warehouse.sdf` | `worlds/` 안의 파일 이름 또는 절대 경로 |
| `gui` | `true` | `false`면 GUI 없이 서버만 실행 |
| `verbosity` | `2` | Gazebo 로그 레벨 (0~4) |

## 실행

```bash
# 컨테이너 안에서
colcon build --packages-select amr_gazebo && source install/setup.bash
ros2 launch amr_gazebo world.launch.py              # GUI 포함 (gui 프로필의 noVNC 화면에서)
ros2 launch amr_gazebo world.launch.py gui:=false   # 서버만
```

## 테스트

```bash
colcon test --packages-select amr_gazebo
```

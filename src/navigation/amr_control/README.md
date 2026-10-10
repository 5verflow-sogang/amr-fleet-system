# amr_control

- 담당: 이진
- 빌드 타입: ament_cmake (C++17)
- 책임: Pure Pursuit/Stanley 경로 추종, PID 속도 제어, 속도 프로파일링

## 구성

| 위치 | 내용 |
| --- | --- |
| `include/amr_control/core/`, `src/core/` | ROS 비의존 코어 (라이브러리 `amr_control_core`). 경로 기하·CTE, Pure Pursuit, Stanley, 추종기, 선속도 PID, `cte.csv` 기록. rclcpp·메시지 타입을 include하지 않는다 |
| `src/path_tracker_node.cpp` | 얇은 ROS 노드. 메시지 변환·제어 주기·`cte.csv` 파일만 맡는다 |

식과 파라미터 근거는 [docs/algorithms/navigation.md](../../../docs/algorithms/navigation.md) 5절.

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `cmd_vel_nav` | `geometry_msgs/msg/Twist` | 게이팅 전 속도 명령 (→ `amr_zone_client` → `amr_safety` → `cmd_vel`) |
| Subscribe | `plan` | `nav_msgs/msg/Path` | 따라갈 경로 |
| Subscribe | `odom` | `nav_msgs/msg/Odometry` | 선속도, 경로와 같은 프레임이면 자세도 (다르면 TF) |

Nav2 `controller_server`도 `cmd_vel_nav`를 내므로 둘을 같이 띄우지 않는다. Nav2 플러그인화는 아직 안 했다.
전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 파라미터

[config/path_tracker.yaml](config/path_tracker.yaml). `controller: pure_pursuit | stanley`로 방식을 고른다.
`velocity_pid.enabled: true`면 추종기 선속도 뒤에 선속도 PID를 건다(기본 끔).
`cte_csv_path`를 주면 하네스 `cte.csv` 형식(`timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment, robot`)으로 기록한다.

## 실행

```bash
ros2 launch amr_control path_tracker.launch.py namespace:=amr_01
ros2 launch amr_control path_tracker.launch.py controller:=stanley cte_csv_path:=/tmp/cte.csv
```

## 테스트

```bash
colcon test --packages-select amr_control
```

- `test_path_geometry`: 투영·부호 있는 CTE·탐색 창·호 길이·곡률, Pure Pursuit·Stanley 식
- `test_path_tracker`: 두 방식 폐루프 (직선 수렴, 횡방향 오프셋 출발, 원호 CTE 상한, 경로 끝 정지, 제자리 회전, 각속도 한계, 재계획)
- `test_pid`: 계단 응답 정착, 상·하한 포화 시 windup 없음, 게인 0 통과, 측정값 미분
- `test_cte_log`: `cte.csv` 헤더·행 형식, 구간 분류

# 런북

Docker로 시스템을 빌드·실행·종료하는 방법.

> 현재 `sim`, `robots`, `fleet`, `perception`은 자리만 잡아 둔 상태다. 컨테이너는 뜨지만 TODO 메시지를 출력하고 대기만 한다. 각 패키지의 launch 파일이 준비되면 [docker-compose.yml](../docker-compose.yml)의 `command`를 교체한다.

## 준비

- Docker (macOS·Windows는 Docker Desktop)
- GPU를 쓸 경우: NVIDIA 드라이버 + nvidia-container-toolkit (Linux)

## 서비스

| 서비스 | 역할 | 기본 실행 |
| --- | --- | --- |
| `sim` | Gazebo 시뮬레이터 (창고 월드) | O |
| `robots` | 로봇 스폰, 로봇별 위치 추정·주행 | O |
| `fleet` | 작업 할당, 다중 로봇 교통 관리 | O |
| `perception` | 카메라 기반 객체 인식 | O |
| `dev` | 개발용 셸 | X (`run`으로 실행) |
| `gui`, `gui-proxy` | 브라우저로 보는 RViz2 | X (`--profile gui`) |

## 빌드

```bash
docker compose build          # 처음 한 번, 또는 Dockerfile·package.xml이 바뀌었을 때
```

## 실행

```bash
docker compose up -d          # 시스템 전체 (sim, robots, fleet, perception)
docker compose ps             # 상태 확인
docker compose logs -f fleet  # 서비스 로그 보기
docker compose down           # 종료
```

### GPU 사용 (NVIDIA)

`perception`에 GPU를 붙인다. GPU가 없는 PC에서는 이 옵션 없이 실행한다.

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d
```

### 개발 셸

```bash
docker compose run --rm dev
colcon build
```

`./src`가 컨테이너에 마운트되어 있어 호스트에서 고친 코드가 바로 반영된다.

### GUI (RViz2)

```bash
docker compose --profile gui up -d
# 브라우저: http://localhost:6080/vnc.html  (비밀번호: ros)
docker compose --profile gui down
```

화면 안의 터미널이나 `docker compose exec gui bash`에서 `rviz2`를 실행한다. 그래픽을 CPU로 그리므로 느릴 수 있다.

## 트레이싱 (ros2_tracing)

연구 1의 지연 분해에 쓴다. LTTng와 ros2_tracing은 이미지(`base`)에 들어 있다. 설치 검증은 [tracing-check.yml](../.github/workflows/tracing-check.yml) 워크플로가 한다.

- Humble 바이너리의 `tracetools`는 LTTng 없이 빌드돼 있어 `ros2 run tracetools status`가 `Tracing disabled`를 낸다. 이미지는 같은 버전을 LTTng와 함께 빌드해 `/opt/tracing_ws`에 오버레이한다 ([build_tracetools.sh](../docker/scripts/build_tracetools.sh)).
- 그래서 `/opt/ros/humble/setup.bash` 대신 **`/opt/tracing_ws/install/setup.bash`를 source**한다 (`dev` 셸의 `.bashrc`는 이미 그렇게 되어 있다). compose 서비스의 `command`에서 트레이스할 때도 이걸 먼저 source한다.
- 컨테이너에서는 **사용자 공간(UST) 이벤트만** 수집한다. 커널 이벤트는 호스트 커널에 lttng-modules가 있어야 해서 쓰지 않는다.
- 트레이스하는 노드와 같은 컨테이너에서 세션을 연다. 세션 데몬과 노드가 공유 메모리로 통신한다.
- 세션을 먼저 시작하고 노드를 띄운다.

```bash
docker compose run --rm dev
source /opt/tracing_ws/install/setup.bash
ros2 run tracetools status    # Tracing enabled

lttng-sessiond --daemonize
lttng create demo --output=/tmp/tracing/demo
lttng enable-event -u 'ros2:*'
lttng add-context -u -t vpid -t vtid -t procname
lttng start
# ... 트레이스할 노드 실행 ...
lttng stop && lttng destroy

# 이벤트 종류별 개수
babeltrace /tmp/tracing/demo | grep -oE 'ros2:[a-z_]+' | sort | uniq -c
```

- `ros2 trace -k`(커널 이벤트 끔)도 같은 일을 하지만 Humble에서는 시작·종료 때 Enter를 눌러야 해서 스크립트에는 위 `lttng` 명령을 쓴다.
- launch로 띄울 때는 `tracetools_launch.action.Trace`를 launch 파일에 넣으면 노드와 함께 세션이 열리고 닫힌다 (`events_kernel=[]`).
- 수집되는 주요 이벤트: `rclcpp_publish`(발행), `callback_start`·`callback_end`(구독·타이머 콜백). 검증은 rclcpp 데모 노드로 했다. rclpy 노드(Fleet 서버, 장애주입기)에서 어떤 이벤트가 남는지는 따로 확인한다.

## 환경 변수

| 변수 | 기본값 | 설명 |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `0` | 같은 네트워크의 다른 PC와 토픽이 섞이면 다른 값으로 바꾼다 |
| `VNC_PASSWORD` | `ros` | GUI 접속 비밀번호 |
| `GUI_RESOLUTION` | `1600x900` | GUI 화면 해상도 |

예: `ROS_DOMAIN_ID=7 docker compose up -d`

## 문제 해결

| 증상 | 원인 · 해결 |
| --- | --- |
| `could not select device driver "nvidia"` | GPU가 없거나 nvidia-container-toolkit이 없는 PC에서 GPU 옵션을 줬다. `docker-compose.gpu.yml` 없이 실행한다 |
| 다른 팀원 로봇의 토픽이 보인다 | `ROS_DOMAIN_ID`를 서로 다르게 설정한다 |
| `localhost:6080` 접속이 안 된다 | `docker compose --profile gui ps`로 `gui`, `gui-proxy`가 둘 다 떠 있는지 확인한다 |
| 코드 의존성을 추가했는데 반영이 안 된다 | `package.xml`에 적은 뒤 `docker compose build`로 이미지를 다시 빌드한다 |

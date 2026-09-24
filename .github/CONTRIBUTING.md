# 기여 가이드

5명이 병렬로 작업하는 레포다. 디렉토리 구조와 인터페이스가 흐트러지지 않도록 아래 규칙을 지킨다. 설계 배경은 [docs/architecture.md](../docs/architecture.md)를 참고한다.

## 1. 브랜치 전략 (Git Flow)

| 브랜치                                                        | 용도                       | 분기 원천 | 머지 대상             |
| ------------------------------------------------------------- | -------------------------- | --------- | --------------------- |
| `main`                                                        | 제출·발표 가능한 안정 버전 | —         | —                     |
| `dev`                                                         | 통합 개발 브랜치           | `main`    | `main` (release 경유) |
| `feat/<영역>-<설명>`                                          | 기능 개발                  | `dev`     | `dev`                 |
| `fix/<영역>-<설명>`                                           | 버그 수정                  | `dev`     | `dev`                 |
| `docs/`, `test/`, `refactor/`, `chore/`, `ci/`, `experiment/` | 성격별 작업                | `dev`     | `dev`                 |
| `release/<버전>`                                              | 제출 전 안정화             | `dev`     | `main`, `dev`         |
| `hotfix/<설명>`                                               | `main` 긴급 수정           | `main`    | `main`, `dev`         |

- 브랜치 이름은 소문자·숫자·`-`·`.`만 쓴다. 예: `feat/nav-astar`, `fix/fleet-deadlock-timeout`
- `<영역>`은 `src/` 아래 도메인 이름을 쓴다: `interfaces`, `sim`, `loc`, `nav`, `perception`, `task`, `fleet`, `obs`, `bringup`, `infra`, `harness`, `dashboard`
- `main`과 `dev`에 직접 push하지 않는다. 모든 변경은 PR로 들어간다.

## 2. 커밋 · PR 제목 (Conventional Commits)

```
<type>(<scope>): <설명>
```

- type: `feat` `fix` `refactor` `perf` `docs` `test` `build` `ci` `chore` `style` `revert`
- scope: 선택. 패키지나 영역 이름. 예: `feat(amr_planning_core): A* 휴리스틱 추가`
- PR은 squash merge하므로 **PR 제목**이 최종 커밋 메시지가 된다.

## 3. PR 규칙

- 리뷰 승인 1명 이상. 변경한 패키지의 담당자(각 패키지 README의 "담당")를 리뷰어로 지정한다.
- CI 통과
- PR 하나에는 한 가지 목적만 담는다. 여러 도메인을 동시에 고쳐야 하면 인터페이스 PR을 먼저 분리한다.
- 성능 수치가 바뀌는 PR은 템플릿의 "성능 개선 / 달성 사항"에 Before/After를 적는다.

## 4. 디렉토리 규칙

### 4-1. 최상위 구조는 고정한다

```
src/            ROS2 패키지만 둔다 (도메인/패키지 2단계)
docker/         Dockerfile, Grafana 대시보드 등 컨테이너 자원
tools/          ROS 패키지가 아닌 개발 도구 (harness 등)
experiments/    연구 실험 시나리오 설정과 분석 결과
dashboard/      웹 프론트엔드 대시보드 (ROS 패키지 아님, COLCON_IGNORE)
docs/           설계·알고리즘·API·리포트 문서
.github/        CI, 템플릿, 기여 가이드
```

최상위 `config/`, `launch/`, `scripts/`는 만들지 않는다. 설정과 launch는 각 패키지 안에 둔다.

### 4-2. 패키지 추가

- 위치: `src/<도메인>/amr_<이름>/`. 이름은 항상 `amr_` 접두어, snake_case.
- 기존 패키지를 복사하지 말고 같은 도메인의 골격 구조를 따른다.
- 새 패키지는 README의 담당·책임·제공 인터페이스를 채우고, [docs/architecture.md](../docs/architecture.md) 패키지 표에 추가한다.

### 4-3. 패키지 내부 구조

패키지 역할에 필요한 폴더만 둔다. 아래는 가능한 전체 구성이고, 골격은 이미 패키지별로 정리돼 있다.

- 노드를 띄우는 패키지: `launch/`, `config/`
- ROS 비의존 라이브러리(`*_core`): 모듈과 `test/`만
- Nav2 플러그인(`amr_nav2_plugins`): 코드와 플러그인 XML만. 파라미터는 `amr_bringup/config`의 Nav2 설정에 둔다
- 자원 전용 패키지(`amr_description`, `amr_gazebo`): `urdf/`·`worlds/`·`models/` 등 자원 폴더와 `launch/`·`config/`

나중에 폴더가 더 필요해지면 만들면 된다. `setup.py`·`CMakeLists.txt`가 폴더가 있을 때 자동으로 설치한다.

Python(`ament_python`):

```
amr_xxx/
├── amr_xxx/          # 모듈 (노드는 *_node.py)
├── launch/           # *.launch.py
├── config/           # *.yaml (모든 파라미터)
├── test/             # pytest
├── resource/amr_xxx
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

C++(`ament_cmake`, C++17):

```
amr_xxx/
├── include/amr_xxx/
├── src/
├── launch/
├── config/
├── test/             # gtest
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 4-4. 의존성 방향

- `*_core` 패키지는 ROS 비의존 라이브러리다. `rclpy`, `rclcpp`, 메시지 타입을 import하지 않는다. 알고리즘은 여기에 두고 단위 테스트한다.
- 노드 패키지는 같은 도메인의 `*_core`와 `amr_msgs`에만 코드 의존한다.
- **다른 도메인과는 토픽·서비스·액션으로만 통신한다.** 다른 도메인 패키지의 Python 모듈이나 C++ 헤더를 직접 import/include하지 않는다.
- `amr_bringup`만 모든 패키지의 launch를 include할 수 있다. 다른 패키지는 bringup에 의존하지 않는다.

### 4-5. 설정과 launch

- 하드코딩 금지. 튜닝 가능한 값은 모두 `config/*.yaml`로 뺀다. 파라미터를 바꿀 때 재빌드가 필요 없어야 한다.
- 패키지마다 자기 launch를 제공하고, 전체 조립은 `amr_bringup`에서만 한다. 공용 launch 파일을 여럿이 고치지 않는다.
- 다중 로봇 대응: 노드는 네임스페이스를 가정하지 말고 상대 토픽 이름(`odom`, `scan`)을 쓴다. 네임스페이스는 bringup이 부여한다.

### 4-6. Mock

다른 사람 작업을 기다리지 않도록, 자기 패키지의 출력 토픽을 흉내 내는 mock 노드를 먼저 제공한다. 실행 파일 이름은 `<노드>_mock`, launch는 `launch/mock.launch.py`로 통일한다.

## 5. 인터페이스 변경 절차 (`amr_msgs`)

1. 메시지를 추가·변경하는 PR을 **단독으로** 올린다. 구현 코드와 섞지 않는다.
2. 해당 메시지를 발행·구독하는 담당자 전원의 승인을 받는다.
3. 기존 필드 삭제·타입 변경은 금지한다. 필요하면 새 메시지를 만들고 이전 메시지는 deprecated로 표시한다.
4. [docs/api.md](../docs/api.md)를 같은 PR에서 갱신한다.

## 6. 커밋하지 않는 것

- `build/`, `install/`, `log/` (colcon 산출물)
- 모델 가중치(`*.pt`, `*.onnx`, `*.engine`), rosbag(`*.db3`, `*.mcap`), 대용량 실험 원자료
  - 가중치는 다운로드 스크립트나 릴리스 자산으로 관리하고, 실험 원자료는 공유 드라이브에 두고 경로만 기록한다.
- `node_modules/`, 프론트엔드 빌드 산출물(`dist/`)
- 개인 IDE 설정, 비밀 정보(토큰·키)

## 7. 로컬 빌드 · 테스트

```bash
# 컨테이너 안, 레포 루트에서
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test && colcon test-result --verbose

# 내 패키지만
colcon build --packages-up-to amr_xxx
colcon test --packages-select amr_xxx
```

`colcon build` 경고 0이 원 명세서 요구사항이다. CI가 경고를 에러로 처리한다.

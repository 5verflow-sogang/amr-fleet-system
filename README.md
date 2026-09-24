# amr-fleet-system

Gazebo 시뮬레이션 기반 지능형 물류 AMR Fleet 시스템. ROS2 Humble 위에서 SLAM·경로계획·제어·AI 인지·작업 수행·다중 로봇 Fleet Management를 End-to-End로 구현한다.

> 상태: 골격 단계 (P0). 설치·실행 방법은 구현에 맞춰 채운다.

## 문서

| 문서 | 내용 |
| --- | --- |
| [docs/project-spec.md](docs/project-spec.md) | 프로젝트 명세 (원 명세서 + 팀 확장, 연구 계획, 일정) |
| [docs/architecture.md](docs/architecture.md) | 아키텍처 전략, 패키지 구성, 다이어그램 |
| [docs/api.md](docs/api.md) | 노드별 토픽·서비스·액션, 이름 규칙, TF 트리 |
| [.github/CONTRIBUTING.md](.github/CONTRIBUTING.md) | 브랜치·커밋·디렉토리 규칙 |

## 레포 구조

```
src/            ROS2 패키지 (interfaces, simulation, localization, navigation,
                perception, task, fleet, observability, bringup)
docker/         Dockerfile, Grafana 설정
tools/harness/  회귀 테스트 하네스
experiments/    연구 1·2 실험
dashboard/      웹 대시보드 (선택)
docs/           설계·알고리즘·API·리포트
```

## 빠른 시작

```bash
# TODO: 구현 후 갱신
docker compose --profile dev run --rm dev
colcon build
```

## 개발 환경

- Ubuntu 22.04, ROS2 Humble, Gazebo Fortress
- Python 3.10+ / C++17
- Docker + nvidia-container-toolkit (GPU 추론 시)

## 팀

| 이름 | GitHub | 담당 |
| --- | --- | --- |
| 이진 (팀장) | [@wjddlwls](https://github.com/wjddlwls) | 경로 계획, 모션 제어 |
| 하연 | [@noeyish](https://github.com/noeyish) | Fleet Management, 인프라·CI |
| 민지 | [@alswl722](https://github.com/alswl722) | SLAM, 위치 추정, 회귀 하네스 |
| 지수 | [@xltuu](https://github.com/xltuu) | 시뮬레이션·로봇 모델, SLAM 공동, 작업 수행(BT·도킹) |
| 연우 | [@ye0n-w0](https://github.com/ye0n-w0) | AI 인지(YOLO·3D 변환), 동적 장애물 추적·회피 |

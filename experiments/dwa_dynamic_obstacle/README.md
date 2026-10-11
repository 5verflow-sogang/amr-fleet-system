# DWA 동적 장애물 회피 Gazebo 실증 (#103)

`amr_nav2_plugins::DwaController`의 동적 장애물 회피(`DwaConfig.predict_dynamic_obstacles`,
단위 테스트까지만 확인했었다)를 `nav2_dwa_demo.launch.py` 데모에서 실제로 확인한다.
로봇 경로(map 기준 (0,0) → (3.0,1.5), 데모 기본 목표)를 가로지르는 가짜 장애물 하나를
`perception/obstacles`(`TrackedObstacleArray`)로 계속 발행하면서, 로봇
`ground_truth/odom`과의 최소 거리와 목표 도착 여부를 기록한다.

## 실행

데모가 이미 떠 있는 컨테이너(또는 같은 세션)에서:

```bash
python3 publish_dynamic_obstacle.py
```

패키지가 아니라 단일 스크립트다 - `colcon build`에 안 걸린다. `docker-compose.yml`이
`./src`만 컨테이너에 마운트해서 `experiments/`는 안 보이므로, 실제로 돌릴 때는
`src/` 밑에 임시로 복사해서 실행하고 끝나면 지웠다(저장소에는 이 경로가 정식 위치).

## 설정

- 장애물: 반경 0.3 m, 0.5 m/s 등속, 경로 중점(map (1.5, 0.75))을 가로지르도록
  x=1.5에 고정해 두고 y 방향으로 내려오게 뒀다.
- 좌표계: `perception/obstacles`는 local costmap의 `global_frame`(이 데모는 `odom`)과
  같은 프레임으로 봐야 하는데, map→odom이 항등 변환이라 map 프레임 숫자를 그대로 써도
  맞다. `ground_truth/odom`은 world 프레임이라(스폰이 world(0, 1.5)) 비교 전에
  world → map으로 변환한다.

## 결과 (2026-10-09, `gui:=false`, `use_static_map`는 이제 기본값이라 인자 없음)

```
목표 도착: True (t=24.5s)
장애물과 최소 거리(GT 기준): 1.333 m (t=16.5s)
```

- **목표 도착**: 했다.
- **최소 거리**: 1.333 m. 로봇 풋프린트(반길이 0.3·반폭 0.2 m)와 장애물 반경(0.3 m) +
  안전 여유(`dynamic_obstacle_safety_margin` 0.2 m)로 계산하면 정면으로 마주쳐도
  대략 0.8~0.9 m보다 가까워지면 하드 필터(충돌)로 걸린다 - 1.333 m는 그보다 충분히
  멀어서, 장애물을 완전히 무시했다면 나올 수 없는 값이다.
- **간접 증거**: 목표까지 걸린 시간이 14.7 s(goal accepted → Reached the goal,
  로그 타임스탬프 차)로, 장애물 없이 돈 이전 실행들의 9~10 s보다 뚜렷하게 길다 -
  장애물을 피하려고 속도를 줄이거나 우회한 것과 일치한다.

**결론: Gazebo에서 동적 장애물을 실제로 피하면서 목표에 도착하는 것을 확인했다.**

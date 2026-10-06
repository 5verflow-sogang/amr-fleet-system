# research2

연구 2 (후보 A: 적재 질량 RL, 연우 주도). 주제와 실험 설계는 [명세서 4장](../../docs/project-spec.md)을 따른다.

## 학습 환경

ROS와 무관한 Python 환경이다. 의존성 버전은 `uv.lock`으로 고정해 누구의 노트북이든 같은 버전으로 학습한다.

```bash
cd experiments/research2
uv sync                                       # .venv 생성, uv.lock 그대로 설치
uv run python scripts/smoke_ppo.py            # 설치 확인: CartPole PPO 학습 속도
```

| 플랫폼 | torch | 비고 |
| --- | --- | --- |
| macOS | PyPI 기본 | |
| Linux (학교 서버, CI) | CPU 전용 (`download.pytorch.org/whl/cpu`) | CUDA를 빼서 설치가 작다. 작은 MLP 정책은 CPU로 학습한다 |

- 의존성 추가는 `uv add <패키지>`로 하고 `pyproject.toml`과 `uv.lock`을 같이 커밋한다. `pip install`로 따로 깔지 않는다.
- 학습 결과(체크포인트, TensorBoard 로그)는 `raw/` 아래에 둔다. git에서 무시된다.
- 스레드 수는 `OMP_NUM_THREADS`로 정한다. 학교 서버에서는 꼭 2로 둔다([runbook](../../docs/runbook.md) "RL 학습").

## 참고 속도 (10/5, `smoke_ppo.py`, 스레드 2)

| 머신 | steps/s |
| --- | --- |
| Mac (Apple Silicon) | 약 7,300 |
| GCP `n2d-standard-4` (Linux, CPU torch) | 약 1,600 |
| 학교 GPU 서버 | 미측정. 단일 코어 속도가 Mac의 약 절반이라 더 느릴 것으로 본다 |

- Linux 설치 확인(10/5, GCP Ubuntu 22.04): `uv sync --locked` 약 4초, torch `2.14.1+cpu`(CUDA 없음), `.venv` 약 1GB
- 클라우드 CPU는 Mac보다 4배 이상 느리다. 학습은 노트북에서 한다

## v0 학습 환경 (`payload_nav/`)

명세서 4장(D7)의 2D 환경이다. 동역학은 이진의 제동 모델(#55)과 같은 식·기본값이다.

```bash
uv run --frozen --with pytest pytest tests                         # 환경 검사
uv run python scripts/train_ppo.py --variant proposed --steps 200000   # 파이프라인 확인 (짧게)
uv run python scripts/train_ppo.py --variant r2 --seed 0
uv run python scripts/evaluate.py raw/proposed_s0_* raw/r2_s0_* raw/r1_s0_*
uv run python scripts/evaluate.py raw/proposed_s0_* --mass-obs none        # 질량 정보 없음
uv run python scripts/evaluate.py raw/proposed_s0_* --mass-obs scale:1.3   # 질량 정보 +30% 오차
uv run tensorboard --logdir raw
```

| 파일 | 내용 |
| --- | --- |
| `config/env_v0.toml` | 보상·시나리오·PPO 설정과 비교군. 값을 바꿀 때는 복사해 `--config`로 고른다 |
| `payload_nav/dynamics.py` | 바퀴 단위 토크 한계(질량 의존) + DiffDrive 회전 가속 한계, 차동 구동 적분, 충돌 판정 |
| `payload_nav/scenarios.py` | 직선 통로(폭 1.2~2.0 m)에서 S1 돌발 장애물(가로지름·멈춰 섬) + S2 정밀 정지. 시나리오는 질량과 무관하게 뽑는다 |
| `payload_nav/env.py` | gymnasium 환경. 관측 48차원(경로 오차·벽 거리·남은 시간·질량 값 + 앎 여부 포함) |
| `payload_nav/evaluation.py` | 에피소드 1회 실행과 지표 (학습 콜백·평가 공용) |

**환경 규칙**

- 돌발 장애물은 경로상 출현 지점(`trigger_x`)을 지날 때 나타난다. 시간이 아니라 위치 기준이라 모든 정책이 같은 곳에서 같은 장애물을 만난다
- 출현 지점은 25 kg으로 반응 지연(0.1 s) 뒤 초기 속도에서 급정지했을 때 멈추는 거리 이후에 둔다. 미리 속도를 줄이면 모든 학습 질량에서 피할 수 있다
- 통로가 좁으면 멈춰 선 장애물을 비켜 갈 수 없어 장애물 앞에서 서야 한다(`popout_blocked`)
- 벽에 닿아도 충돌로 끝난다. 시간 초과(30 s)는 남은 시간을 관측하므로 실패(terminated)로 끝내고 −10을 준다

**평가 CSV 주요 열**

| 열 | 뜻 |
| --- | --- |
| `outcome` | success / collision / timeout / overshoot |
| `collision_with`, `impact_speed` | 충돌 대상(obstacle / wall)과 충돌 순간 속도 |
| `min_clearance`, `near_miss`, `min_ttc` | 장애물과의 최소 거리, 0.3 m 미만 여부, 최소 TTC |
| `popout_kind` | crossing(가로지름) / standing(멈춰 섬) |
| `popout_stoppable` | 나타난 순간 실제 질량으로 반응 지연 0.1 s 뒤 급정지해 설 수 있었는지 |
| `popout_blocked` | 장애물 옆으로 비켜 갈 공간이 없었는지 |
| `popout_result` | stopped(장애물 앞에서 멈춤) / passed(멈추지 않고 지나감) / collision |
| `popout_stop_distance`, `popout_stop_margin` | 실제 제동 거리, 제동 거리 여유(나타날 때 간격 − 제동 거리). stopped일 때만 |
| `jerk_rms`, `jerk_max` | 출발 직후 2스텝을 뺀 jerk |

- 비교군: `r1`(0 kg 고정), `r2`(질량 무작위, 질량 모름), `proposed`(질량 무작위 + 관측, 학습 중 20%는 질량을 가림)
- 체크포인트는 학습 질량 검증 환경의 성공률로 고른다(`best_*`). 평가 seed는 학습·검증과 겹치지 않는다
- 평가 질량 기본값: 학습 0·2·10·25 kg + 미학습 15·30 kg
- pytest는 `uv.lock`을 바꾸지 않으려고 `--with`로만 쓴다

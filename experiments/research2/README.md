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

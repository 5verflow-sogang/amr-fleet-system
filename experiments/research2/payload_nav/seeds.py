"""시드 규칙 (실험 설계 2절). 학습 환경·검증·평가 시나리오의 시드 범위가 서로 겹치지 않게 나눈다.

| 용도 | 시드 |
| --- | --- |
| PPO 신경망 초기화 | 학습 시드 k 그대로 |
| 학습 환경 | TRAIN_ENV_SEED_BASE + k · TRAIN_ENV_SEED_STRIDE + i (i = 병렬 환경 번호) |
| 검증 시나리오 (체크포인트 선택) | VAL_SEED_BASE + 에피소드 번호 |
| 평가 시나리오 | EVAL_SEED_BASE + 에피소드 번호 |
"""

VAL_SEED_BASE = 10_000
EVAL_SEED_BASE = 100_000
TRAIN_ENV_SEED_BASE = 1_000_000
TRAIN_ENV_SEED_STRIDE = 100   # 학습 시드 하나가 쓰는 환경 시드 칸 수 (병렬 환경 수 상한)


def train_env_seed(train_seed: int, n_envs: int) -> int:
    """학습 시드 → 병렬 환경 0번의 시드. SB3 make_vec_env가 환경 i에 이 값 + i를 준다.

    학습 시드를 그대로 넘기면 시드 k와 k+1이 환경 시드 n_envs − 1개를 공유해, 서로 다른 시드가
    거의 같은 시나리오·질량 흐름으로 학습한다.
    """
    if train_seed < 0:
        raise ValueError(f'학습 시드는 0 이상이어야 한다: {train_seed}')
    if not 0 < n_envs <= TRAIN_ENV_SEED_STRIDE:
        raise ValueError(f'n_envs는 1~{TRAIN_ENV_SEED_STRIDE}여야 한다: {n_envs}')
    return TRAIN_ENV_SEED_BASE + train_seed * TRAIN_ENV_SEED_STRIDE

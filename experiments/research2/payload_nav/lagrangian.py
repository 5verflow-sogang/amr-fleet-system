"""제약 RL (v3): PPO-Lagrangian. 충돌을 보상 벌점 대신 제약 "학습 분포 에피소드 충돌 비율 ≤ δ"로 다룬다.

    max_π J_r(π)   s.t.   J_c(π) ≤ δ        →   max_π min_λ≥0  J_r(π) − λ (J_c(π) − δ)

- 비용: 환경 info['cost'] (충돌로 끝난 스텝만 1). 롤아웃 중 콜백(_CostRecorder)이 롤아웃 버퍼와 같은 위치에 기록한다.
  SB3 collect_rollouts는 callback.on_step()을 rollout_buffer.add()보다 먼저 부르므로, 그때 buffer.pos가 이 스텝 자리다
- 비용 가치망 V_c: 정책과 따로 둔 MLP. 비용 GAE(cost_gamma, cost_gae_lambda)의 목표로 학습한다. 저장 파일에는 넣지 않는다
  (평가는 정책만 쓰므로 evaluate.py가 PPO.load로 그대로 읽는다)
- λ: 롤아웃마다 끝난 에피소드의 충돌 비율 → 이동평균 J_c → 로그 공간 PID (Stooke et al. 2020, "Responsive Safety in RL
  by PID Lagrangian Methods"의 PID를 log λ에 건다. PIDLagrangian 설명 참고)
- 정책 손실의 이득: Â = (Â_r' − λ·Â_c') / (1 + λ). Â_r'은 보상 이득을 표준화한 값, Â_c'은 비용 이득에서 평균만 뺀 값이다.
  충돌이 드물어(δ = 1%면 롤아웃당 한두 번) 비용 이득을 표준화하면 비용 가치망 잡음이 단위 분산으로 커져 정책 경사에 섞인다.
  그래서 Â_c'은 확률 단위 그대로 두고, λ가 "충돌 확률 1당 보상 이득 표준편차 몇 배"를 뜻하게 한다.
  보상 가치망은 그대로 보상 GAE로 학습한다 (rollout_buffer.returns는 바꾸지 않는다)

λ가 늘 0이면(PID 이득 모두 0, lambda_init 0) Â = 표준화한 Â_r이고, PPO가 미니배치마다 이득을 다시 표준화하므로 일반 PPO와
같은 업데이트가 된다 (tests/test_lagrangian.py). 비용 가치망은 전역 난수를 쓰지 않아 정책·환경 난수 흐름을 바꾸지 않는다.
"""

from typing import List, Optional, Sequence

import numpy as np
import torch as th
from torch import nn
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
from stable_baselines3.common.utils import explained_variance, obs_as_tensor

from .config import ConstraintConfig


def gae(rewards: np.ndarray, values: np.ndarray, episode_starts: np.ndarray, last_values: np.ndarray,
        last_dones: np.ndarray, gamma: float, gae_lambda: float):
    """GAE(λ) 이득과 TD(λ) 목표. SB3 RolloutBuffer.compute_returns_and_advantage와 같은 식이다.

    배열은 (n_steps, n_envs). episode_starts[t]는 스텝 t가 새 에피소드의 첫 스텝인지, last_dones는 마지막 스텝 뒤 종료 여부.
    """
    n_steps = rewards.shape[0]
    advantages = np.zeros_like(rewards, dtype=np.float32)
    last_gae = np.zeros(rewards.shape[1], dtype=np.float32)
    for t in reversed(range(n_steps)):
        if t == n_steps - 1:
            next_non_terminal = 1.0 - last_dones.astype(np.float32)
            next_values = last_values
        else:
            next_non_terminal = 1.0 - episode_starts[t + 1]
            next_values = values[t + 1]
        delta = rewards[t] + gamma * next_values * next_non_terminal - values[t]
        last_gae = delta + gamma * gae_lambda * next_non_terminal * last_gae
        advantages[t] = last_gae
    return advantages, advantages + values


def combine_advantages(adv_r: np.ndarray, adv_c: np.ndarray, lam: float) -> np.ndarray:
    """정책 손실에 쓰는 이득 (Â_r' − λ·Â_c') / (1 + λ). Â_r'은 표준화, Â_c'은 평균만 뺀다 (모듈 설명 참고)."""
    adv_r = (adv_r - adv_r.mean()) / (adv_r.std() + 1e-8)
    return ((adv_r - lam * (adv_c - adv_c.mean())) / (1.0 + lam)).astype(np.float32)


class PIDLagrangian:
    """로그 공간 PID 라그랑지안:  λ = clip(exp(ν + Kp·e + Kd·max(0, Δe)), λ_min, λ_max),  ν ← clip(ν + Ki·e).

    오차 e = ln((J_c + δ) / 2δ)는 J_c = δ에서 0, J_c = 0에서 −ln 2, J_c = 5δ에서 +ln 3이다.
    선형 오차 (J_c − δ)/δ는 아래로 −1, 위로 1/δ − 1(δ = 1%면 99)까지라, 초반 위반으로 λ가 크게 오른 뒤 충돌을 없애도
    내려오는 데 학습 전체보다 긴 롤아웃이 걸린다(Ki = 0.05면 20 → 0에 약 400 롤아웃, 300만 스텝은 약 180 롤아웃).
    그러면 벌점 방식처럼 "늘 조심"으로 굳는다. 로그 공간에서는 λ가 오를 때도 내릴 때도 비율로 움직여 대칭에 가깝다
    (Ki = 0.1이면 충돌 0일 때 롤아웃마다 약 7%씩 줄어 10 → 1에 약 33 롤아웃).

    J_c는 롤아웃 충돌 비율의 이동평균(cost_ema). 끝난 에피소드가 없는 롤아웃은 λ를 바꾸지 않는다.
    ν를 [ln λ_min, ln λ_max]로 자르는 것은 적분 누적(windup) 방지다. 미분 항은 위반이 커질 때만 반응한다(Stooke et al.).
    이득이 모두 0이면 λ는 lambda_init 그대로다(0도 가능. 테스트에서 일반 PPO와 비교할 때 쓴다).
    """

    def __init__(self, c: ConstraintConfig):
        self.c = c
        self.fixed = c.pid_kp == 0.0 and c.pid_ki == 0.0 and c.pid_kd == 0.0
        self.log_integral = float(np.log(min(max(c.lambda_init, c.lambda_min), c.lambda_max)))
        self.cost_ema: Optional[float] = None
        self.prev_error: Optional[float] = None
        self.value = c.lambda_init

    def update(self, cost_rate: Optional[float]) -> float:
        c = self.c
        if cost_rate is None:
            return self.value
        self.cost_ema = cost_rate if self.cost_ema is None else c.cost_ema * self.cost_ema + (1 - c.cost_ema) * cost_rate
        if self.fixed:
            return self.value
        error = float(np.log((self.cost_ema + c.cost_limit) / (2.0 * c.cost_limit)))
        lo, hi = np.log(c.lambda_min), np.log(c.lambda_max)
        self.log_integral = float(np.clip(self.log_integral + c.pid_ki * error, lo, hi))
        derivative = 0.0 if self.prev_error is None else max(0.0, error - self.prev_error)
        self.prev_error = error
        self.value = float(np.clip(np.exp(self.log_integral + c.pid_kp * error + c.pid_kd * derivative),
                                   c.lambda_min, c.lambda_max))
        return self.value


class _CostRecorder(BaseCallback):
    """롤아웃 중 비용을 롤아웃 버퍼와 같은 자리에 적고, 끝난 에피소드의 비용 합을 모은다."""

    def _on_rollout_start(self) -> None:
        self.model._episode_costs = []

    def _on_step(self) -> bool:
        m = self.model
        costs = np.array([info.get('cost', 0.0) for info in self.locals['infos']], dtype=np.float32)
        m._costs[m.rollout_buffer.pos] = costs     # add() 전이라 pos가 이 스텝 자리다
        m._running_cost += costs
        for i, done in enumerate(self.locals['dones']):
            if done:
                m._episode_costs.append(float(m._running_cost[i]))
                m._running_cost[i] = 0.0
        return True


def _mlp(n_in: int, hidden: Sequence[int]) -> nn.Sequential:
    layers: List[nn.Module] = []
    for h in hidden:
        layers += [nn.Linear(n_in, h), nn.Tanh()]   # SB3 MlpPolicy 기본 활성화와 같게
        n_in = h
    layers.append(nn.Linear(n_in, 1))
    return nn.Sequential(*layers)


class PPOLagrangian(PPO):
    """PPO + 비용 가치망 + PID 라그랑지안. 나머지(롤아웃·보상 가치망·클리핑·target_kl)는 SB3 PPO 그대로다."""

    def __init__(self, *args, constraint: ConstraintConfig, **kwargs):
        self.constraint = constraint   # _setup_model(super().__init__ 안에서 불림)이 쓴다
        super().__init__(*args, **kwargs)

    def _setup_model(self) -> None:
        super()._setup_model()
        c = self.constraint
        arch = self.policy_kwargs.get('net_arch', [64, 64])
        hidden = arch['vf'] if isinstance(arch, dict) else arch
        # 전역 torch 난수를 건드리지 않게 따로 떼어 초기화한다 (정책 행동 샘플링 흐름이 일반 PPO와 같게)
        with th.random.fork_rng(devices=[]):
            th.manual_seed(0 if self.seed is None else self.seed)
            self.cost_vf = _mlp(int(np.prod(self.observation_space.shape)), hidden).to(self.device)
        self.cost_vf_optimizer = th.optim.Adam(self.cost_vf.parameters(), lr=c.cost_vf_lr)
        self.lagrangian = PIDLagrangian(c)
        self._costs = np.zeros((self.n_steps, self.n_envs), dtype=np.float32)
        self._running_cost = np.zeros(self.n_envs, dtype=np.float32)
        self._episode_costs: List[float] = []
        self._cost_rng = np.random.default_rng(0 if self.seed is None else self.seed)

    def _init_callback(self, callback, progress_bar: bool = False) -> BaseCallback:
        callback = super()._init_callback(callback, progress_bar)
        wrapped = CallbackList([_CostRecorder(), callback])
        wrapped.init_callback(self)
        return wrapped

    def _cost_values(self, obs: np.ndarray) -> np.ndarray:
        with th.no_grad():
            return self.cost_vf(obs_as_tensor(obs, self.device)).squeeze(-1).cpu().numpy()

    def train(self) -> None:
        c, buf = self.constraint, self.rollout_buffer
        obs = buf.observations.reshape(-1, *self.observation_space.shape).copy()   # get()이 버퍼를 펴기 전에
        values_c = self._cost_values(obs).reshape(self.n_steps, self.n_envs)
        last_values_c = self._cost_values(self._last_obs).reshape(-1)
        adv_c, ret_c = gae(self._costs, values_c, buf.episode_starts, last_values_c,
                           np.asarray(self._last_episode_starts), c.cost_gamma, c.cost_gae_lambda)

        cost_rate = float(np.mean(self._episode_costs)) if self._episode_costs else None
        lam = self.lagrangian.update(cost_rate)
        buf.advantages = combine_advantages(buf.advantages, adv_c, lam)

        self.logger.record('lagrangian/lambda', lam)
        self.logger.record('lagrangian/cost_rate_ema', self.lagrangian.cost_ema if self.lagrangian.cost_ema is not None else float('nan'))
        if cost_rate is not None:
            self.logger.record('lagrangian/cost_rate_rollout', cost_rate)
        self.logger.record('lagrangian/episodes', len(self._episode_costs))
        self.logger.record('lagrangian/cost_explained_variance',
                           float(explained_variance(values_c.flatten(), ret_c.flatten())))

        super().train()
        self.logger.record('lagrangian/cost_vf_loss', self._fit_cost_vf(obs, ret_c.reshape(-1)))

    def _fit_cost_vf(self, obs: np.ndarray, targets: np.ndarray) -> float:
        """비용 가치망을 비용 TD(λ) 목표에 맞춘다. 미니배치 순서는 전용 난수로 섞는다 (전역 난수를 쓰지 않음)."""
        obs_t, tgt_t = obs_as_tensor(obs, self.device), th.as_tensor(targets, device=self.device)
        losses = []
        for _ in range(self.constraint.cost_vf_epochs):
            order = self._cost_rng.permutation(len(targets))
            bs = self.constraint.cost_vf_batch_size
            for start in range(0, len(targets), bs):
                idx = th.as_tensor(order[start:start + bs], device=self.device)
                loss = nn.functional.mse_loss(self.cost_vf(obs_t[idx]).squeeze(-1), tgt_t[idx])
                self.cost_vf_optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.cost_vf.parameters(), self.max_grad_norm)
                self.cost_vf_optimizer.step()
                losses.append(loss.item())
        return float(np.mean(losses))

    def _excluded_save_params(self) -> List[str]:
        # 저장 파일은 일반 PPO와 같은 내용만 담는다 (evaluate.py가 PPO.load로 읽는다)
        return super()._excluded_save_params() + [
            'constraint', 'cost_vf', 'cost_vf_optimizer', 'lagrangian', '_costs', '_running_cost',
            '_episode_costs', '_cost_rng']

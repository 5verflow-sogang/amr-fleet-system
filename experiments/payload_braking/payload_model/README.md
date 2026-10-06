# payload_braking_model

적재 질량에 따른 가속·제동 거리/시간을 구하는 순수 Python 물리 모델. ROS
의존 없음(표준 라이브러리만) — 연우의 2D RL 학습 환경에서 바로 import해서 쓸 수
있다.

## 사용 예시

```python
from payload_braking_model import BrakingModel

model = BrakingModel()  # 지수 URDF(#44) 기본값
distance = model.braking_distance_m(v_mps=1.0, payload_kg=10.0)  # 급정지(a_cmd 없음)
```

RL 환경의 한 제어 스텝(연우 #51 `step(v, w, v_cmd, w_cmd, m, dt) -> (v', w')` 모양):

```python
v, w = model.step(v_mps=1.0, w_radps=0.0, v_cmd_mps=0.0, w_cmd_radps=0.0,
                   payload_kg=10.0, dt_s=0.1)  # -> (0.8458, 0.0)
```

설치: `pip install -e experiments/payload_braking/payload_model` (또는 그 디렉토리 안에서 `pip install -e .`).

## 식

```
a_max = min( 2 * 바퀴토크 / (바퀴반경 * (로봇 질량 + 적재)),  마찰계수 * g )
a     = min(a_cmd, a_max)              # a_cmd 없으면(None) a_max 그대로 - 급정지
제동 거리      = v * t_delay + v**2 / (2 * a)
0->v 도달 시간 = t_delay + v / a        # 가속에도 같은 식
```

- `a_max`는 바퀴 토크 한계와 바닥 마찰 한계(`mu * g`) 중 **작은 쪽**이다 — 적재가
  가벼워 토크가 남아돌아도 바닥이 미끄러지면 그 이상 못 낸다.
- `a_cmd`(명령 감속 한계)를 주면 "normal"(소프트웨어로 감속을 제한하는) 정지,
  안 주면(`None`) 토크 한계까지 그대로 쓰는 "emergency" 급정지다.
- `t_delay`는 명령이 나간 순간부터 실제로 감속이 시작되기까지의 반응 지연이다
  (아래 "파라미터 근거" 참고). **`step()`에는 지연을 안 쓴다** — 아래 "step()"
  절 참고.

### step(): RL 환경의 한 제어 스텝

```
a_limit = min(a_max(질량), a_cmd)      # a_cmd 없으면 a_max 그대로
v' = clip(v + clip(v_cmd - v, -a_limit*dt, a_limit*dt), -2.0, 2.0)
w' = clip(w + clip(w_cmd - w, -3.2*dt, 3.2*dt), -1.5, 1.5)   # 질량 반영 안 함(D7)
```

`braking_distance_m`/`time_to_reach_speed_s`는 "한 번에 멈출 때까지 걸리는
거리·시간"을 닫힌 식으로 구하지만, `step()`은 RL 환경이 **매 타임스텝마다**
부르는 용도라 "지금 한 스텝만큼만" (v, w)를 바꾼다 - 가속·감속 모두 같은 한계
(`max_acceleration_mps2`)를 쓰고, 각속도는 질량과 무관하게 고정 한계
(`angular_accel_max_radps2`, D7 v0 합의)로 바뀐다.

**`step()`은 지연(`t_delay`)이 없다.** `t_delay`는 "명령이 나가고 토크가 실제로
올라오기까지"의 반응 지연인데, 매 스텝 호출되는 `step()`에서 이걸 표현하려면
과거 명령을 몇 스텝 들고 있어야 해서(명령 버퍼) 이 함수의 범위를 벗어난다.
**지연이 필요하면 RL 환경 쪽에서 명령을 `t_delay_s`(기본 0.022 s)만큼 버퍼에
쌓아 몇 스텝 늦게 `step()`에 넘기는 방식으로 처리한다.**

### #51 환경과의 호환성

이 작성 시점(#51 `feat/rl-env` 기준)에는 `experiments/research2/`에 uv 환경
설정과 설치 확인용 `smoke_ppo.py`(CartPole)만 있고, AMR용 `gymnasium.Env`나
실제 `step()` 호출부는 아직 없다 — 그래서 "거기서 쓰는 단위·인자 순서"를
코드로 대조할 대상이 없다. `step(v, w, v_cmd, w_cmd, payload_kg, dt)` 순서와
단위(m/s, rad/s, kg, s)는 이 작업 지시에 적힌 그대로 구현했다. 실제 환경
코드가 올라오면 인자 순서·단위가 이것과 맞는지 다시 확인해야 한다.

## 파라미터 근거

| 파라미터 | 기본값 | 근거 |
| --- | --- | --- |
| `wheel_torque_max_nm` | 3.0 N·m | #42(`experiments/payload_braking`) 실험 권장값을 #44가 그대로 채택 |
| `wheel_radius_m` | 0.1 m | #44 `amr_description/urdf/properties.xacro`의 `wheel_radius` |
| `robot_mass_kg` | 28.9 kg | #44 기준: 몸체 25 + 바퀴 1.5×2 + 캐스터 0.3×2 + 센서(LiDAR 0.2+카메라 0.1). **적재 제외** |
| `floor_friction_mu` | 0.7 | #42 `world/flat_ground.sdf`·물류센터 월드 바닥 마찰 계수(둘 다 0.7로 같음) |
| `gravity_mps2` | 9.81 | 표준 중력가속도 |
| `t_delay_s` | 0.022 s | 아래 "t_delay 보정". `step()`에는 안 씀 |
| `linear_velocity_max_mps` | 2.0 m/s | `nav2_params.yaml` `FollowPath.max_vel_x`·#44 URDF `max_linear_velocity`와 동일. `step()`에서만 씀 |
| `angular_velocity_max_radps` | 1.5 rad/s | `max_vel_theta`·URDF `max_angular_velocity`와 동일. `step()`에서만 씀 |
| `angular_accel_max_radps2` | 3.2 rad/s² | `acc_lim_theta`·URDF `max_angular_acceleration`와 동일. `step()`에서만 씀, 질량 반영 안 함(D7) |

**#42 최소 모델(`experiments/payload_braking/model/`)의 질량은 28 kg**(몸체 25 +
바퀴 1.5×2, 캐스터·센서 질량은 이론값 계산에서 뺐다)로 지수 모델(28.9 kg)보다
0.9 kg 가볍다 — 아래 "검증" 절에서 `robot_mass_kg`를 28.0/28.9 둘 다로 바꿔서
비교한다.

### t_delay 보정

`experiments/payload_braking/results.csv`(#42)의 **급정지(emergency) 40회**
(드라이브 방식 V·T × 적재 0/2/10/25kg × 5회)에서, 실측 제동거리와 지연 없는
이론값(`v**2/(2*a_max)`)의 차이를 전부 구해 평균을 냈다:

```
평균 오차 = 0.0221 m  (표준편차 0.0026 m, n=40)
```

v=1 m/s로 측정했으므로 `거리 = v * t_delay`에서 `t_delay = 거리 / v = 0.0221 s`.
**0.022초로 반올림**해서 기본값으로 뒀다. normal(소프트웨어 감속 제한) 결과는
이 보정에 **쓰지 않았다** — 이유는 "한계" 절 참고.

## 검증

`experiments/payload_braking/results.csv`(#42, 5회 평균)와 `amr_description`
README(#44, 1회 측정)를 모델 예측과 비교했다. 단위 m, 오차 = 실측−예측.

### 급정지 (emergency, `a_cmd` 없음)

| 적재 | V 실측(#42) | T 실측(#42) | 모델(28.0kg) | V 오차 | T 오차 |
| --- | --- | --- | --- | --- | --- |
| 0 kg | 0.2539 | 0.2548 | 0.2553 | −0.0014 | −0.0005 |
| 2 kg | 0.2708 | 0.2744 | 0.2720 | −0.0012 | +0.0024 |
| 10 kg | 0.3371 | 0.3399 | 0.3387 | −0.0016 | +0.0012 |
| 25 kg | 0.4623 | 0.4666 | 0.4637 | −0.0014 | +0.0029 |

전부 **3mm 이내** — `t_delay`를 이 데이터로 직접 맞췄으니 당연한 결과지만,
V·T 양쪽에 똑같이 잘 맞는다는 것은 "같은 토크 한계에 걸려서 멈춘다"는 #42의
결론과 일치한다.

| 적재 | #44 실측(28.9kg) | 모델(28.9kg) | 오차 |
| --- | --- | --- | --- |
| 0 kg | 0.254 | 0.2628 | −0.0088 |
| 10 kg | 0.337 | 0.3462 | −0.0092 |
| 25 kg | 0.462 | 0.4712 | −0.0092 |

**1cm 이내** — #42 데이터로 맞춘 `t_delay`가 0.9kg 더 무거운 실제 로봇(#44)에도
잘 맞는다.

### 일반 정지 (normal, `a_cmd=1.0`)

| 적재 | V 실측(#42) | T 실측(#42) | 모델(28.0kg, a_cmd=1.0) | V 오차 | T 오차 |
| --- | --- | --- | --- | --- | --- |
| 0 kg | 0.5027 | 0.5332 | 0.5220 | −0.0193 | +0.0112 |
| 2 kg | 0.5041 | 0.5362 | 0.5220 | −0.0179 | +0.0142 |
| 10 kg | 0.5035 | 0.5430 | 0.5220 | −0.0185 | **+0.0210** |
| 25 kg | 0.5033 | 0.5569 | 0.5220 | −0.0187 | **+0.0349** |

**T/10kg(2.1cm), T/25kg(3.5cm)가 2cm를 넘는다** — 이유는 "한계" 절.
V는 전부 2cm 이내(−1.8~−1.9cm)지만 마찬가지로 설명이 필요해서 같이 적는다.

## 한계

- **a_cmd로 감속을 제한하면(normal) 적재가 거의 안 드러난다** — 이게 이 모델이
  보여주는 핵심 성질이다: `a_cmd`(여기서는 1.0 m/s²)가 `a_max`(토크 한계)보다
  작으면 `a = a_cmd`로 고정되고, 적재는 `a_max` 계산에만 들어가므로 전혀
  영향이 없다. 테스트 범위(0~25kg)에서 `a_max`는 2.14(0kg) → 1.13(25kg)
  m/s²로 줄어드는데, 25kg에서도 여전히 `a_cmd`(1.0)보다 커서 `a=a_cmd`가
  유지된다 — 다만 여유가 13%까지 줄어, 아래 T/normal 오차가 적재가 클수록
  커지는 것과 무관하지 않다(실제 모터가 한계에 더 가깝게 일하기 때문).
  #42·#44 실측도 "normal 정지는 질량과 거의 무관"이라는 같은 결론이었다.
  **적재가 제동거리에 뚜렷이 드러나길 원하면 `a_cmd`를 비우거나(`None`, 급정지)
  `a_max`에 더 가까운 값으로 둬야 한다.**
- **T/normal의 2cm 초과 오차**: `t_delay`는 "명령 후 토크가 올라오기까지의 고정
  지연"만 나타낸다. 그런데 #42의 T(JointController) 구동은 "normal" 정지를
  **소프트웨어 램프 + 비례(P)제어만으로 그 램프를 추종**하는 방식으로 구현했다
  (`scripts/cmd_vel_to_wheel_torque.py`) — 비례제어만 쓰면 필요한 토크가 클수록
  추종 오차(속도가 목표보다 살짝 뒤처지는 정상상태 오차)가 커지고, 이 오차는
  **적재가 무거울수록(더 큰 토크가 필요할수록) 커진다**. 이건 고정 지연이 아니라
  "질량에 비례해서 커지는 지연"이라 이 모델의 `t_delay`(고정값) 하나로는 표현이
  안 된다 - 그래서 T/normal에서 적재가 커질수록 오차도 커진다(0kg +1.1cm →
  25kg +3.5cm, 거의 선형).
- **V/normal의 일정한 −1.8~1.9cm 오차**: 반대로 V(DiffDrive)의 "normal" 정지는
  실제로 반응 지연이 거의 없다(플러그인이 가속 한계를 속도 명령에 바로 적용한다).
  `t_delay`는 **급정지(emergency, 토크가 바로 포화되는 상황)** 데이터로 맞춘
  값이라, 토크 여유가 큰 normal 정지에 그대로 적용하면 없는 지연을 더해 실제보다
  살짝 더 멀리 간다고 예측한다. 크기가 작고(2cm 이내) 적재와 무관하게 일정해서
  "검증" 기준은 통과하지만, 원인은 다르다.
- **15 kg·30 kg은 외삽이다** — 실측은 0/2/10/25 kg(#42)과 0/10/25 kg(#44)만
  했다. 공식 자체는 질량 범위에 꺾이는 지점이 없어(토크 한계 구간이면 매끄럽게
  반비례) 그 사이·바로 바깥 값은 믿을 만하다고 보지만, 독립적으로 실측하지는
  않았다.
- **바닥 마찰 상한(`mu*g`)은 이 실험에서 한 번도 실제로 걸려본 적이 없다** -
  0~25kg·토크 3N·m 범위에서는 항상 토크 쪽이 더 작은 한계였다(테스트
  `test_max_acceleration_is_friction_limited_when_torque_is_huge`가 공식만
  확인). 아주 무거운 적재(예: 수백 kg)나 토크를 크게 올리는 시나리오라면 마찰
  한계 쪽도 실측으로 확인해야 한다.

## 테스트

```bash
pip install -e ".[test]"
pytest tests/
```

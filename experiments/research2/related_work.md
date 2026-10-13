# 연구 2 관련 선행연구와 차별점

[명세서 4장](../../docs/project-spec.md) 연구 2(적재 질량 변화에 강건한 물류 AMR 강화학습 로컬 플래너)의 선행연구를 다섯 갈래로 정리하고 차별점을 정한다. 명세서 10-2절의 조사 후보를 서지 확인한 결과이며, 14장 "연구 2 선행연구 서지 확인 및 차별점 문장 확정" 항목에 해당한다.

> 상태: 10/12 조사. 아래 논문은 모두 웹 검색으로 제목·저자·발표처·연도를 확인했다(arXiv, 학회 프로시딩, 출판사·기관 저장소 페이지). 표의 "핵심"은 초록 수준에서 확인한 내용이다. 원고에 인용하기 전 본문을 다시 확인한다.

용어는 명세서 4장을 따른다: B1(빈 차 기준 감속 한계 고정 DWA), B2(실제 질량에 맞춘 감속 한계 DWA), R1(0 kg 고정 학습), R2(질량 무작위화, 질량 모름), 제안(질량 무작위화 + 질량 관측, 학습 중 20% 가림), 돌발 장애물, 출현 지점, 앎 여부 플래그.

## 1. 동역학·도메인 무작위화 (sim-to-real)

| 논문 | 핵심 | 우리와의 관계/차이 |
| --- | --- | --- |
| Peng 외, Sim-to-Real Transfer of Robotic Control with Dynamics Randomization (ICRA 2018) | 학습 중 시뮬레이터 동역학(질량·마찰 등)을 무작위화해, 실물 학습 없이 로봇 팔 물체 밀기 정책을 옮겼다. 보정 오차에도 강건했다 | R2·제안의 질량 무작위화가 이 방식이다. 우리는 실물 이전이 아니라 "적재 질량이 바뀌는 운용 조건"을 무작위화 대상으로 삼고, 무작위화 값을 관측으로 줄지(제안) 말지(R2)를 비교한다 |
| Tobin 외, Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World (IROS 2017) | 렌더링(질감·조명)을 무작위화해 시뮬레이션 영상만으로 실세계 물체 위치 추정기를 학습했다 | 도메인 무작위화의 출발점으로 인용한다. 우리 무작위화는 시각이 아니라 동역학(토크 한계에 따른 가감속)이다 |

## 2. 물리 파라미터 적응·특권 정보

| 논문 | 핵심 | 우리와의 관계/차이 |
| --- | --- | --- |
| Yu 외, Preparing for the Unknown: Learning a Universal Policy with Online System Identification (RSS 2017) | 여러 동역학에서 학습한 범용 정책(UP)에 최근 상태·행동 이력으로 파라미터를 추정하는 온라인 시스템 식별(OSI)을 붙였다 | 제안 방법의 "질량을 관측에 넣는 범용 정책"과 구조가 같다. 우리는 질량을 추정하지 않고 `payload_mass` 파라미터로 받되, 그 값이 틀리거나(±30%) 없을 때를 따로 평가한다 |
| Kumar 외, RMA: Rapid Motor Adaptation for Legged Robots (RSS 2021) | 기본 정책 + 적응 모듈. 학습 때는 환경 파라미터(적재 포함)를 특권 정보로 쓰고, 배치 때는 과거 이력으로 그 잠재 표현을 추정한다. 4족 A1에서 미세조정 없이 동작 | 적재 변화 적응의 대표 사례지만 4족 보행이다. 우리 R2는 이력 4스텝만 보는 "질량 모름" 기준선이고, 이력 기반 추정(RecurrentPPO)은 명세서 4-6절 후속 실험으로 남겨 두었다 |
| Lee 외, Learning Quadrupedal Locomotion over Challenging Terrain (Science Robotics 2020) | 지형·접촉 정답을 보는 교사 정책을 고유수용 감각만 쓰는 학생 정책으로 증류했다 | 특권 정보 학습의 근거로 인용한다. 우리는 증류 없이 질량 칸을 (0, 0)으로 가리는 방식(학습 중 20%)으로 "정보 없음" 입력을 학습 분포에 넣는다 |

## 3. DRL 로컬 플래너·동적 장애물 회피

| 논문 | 핵심 | 우리와의 관계/차이 |
| --- | --- | --- |
| Schulman 외, Proximal Policy Optimization Algorithms (arXiv 1707.06347, 2017) | 클리핑한 대리 목적함수로 같은 배치를 여러 epoch 갱신하는 정책 경사 방법 | 학습 알고리즘(명세서 4-3절) |
| Chen 외, Decentralized Non-communicating Multiagent Collision Avoidance with Deep Reinforcement Learning (CADRL, ICRA 2017) | 가치망을 오프라인 학습해 통신 없이 다중 에이전트 충돌 회피. ORCA보다 경로가 26% 넘게 빨랐다 | DRL 충돌 회피의 출발점. 로봇 동역학은 단순화돼 있고 적재 변화는 다루지 않는다 |
| Everett 외, Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning (GA3C-CADRL, IROS 2018) | LSTM으로 주변 에이전트 수에 무관한 입력을 만들어 동적 에이전트 사이를 주행 | 장애물 입력을 "가까운 3개 + 존재 여부"로 고정 길이화한 우리 설계와 대비된다 |
| Long 외, Towards Optimally Decentralized Multi-Robot Collision Avoidance via Deep Reinforcement Learning (ICRA 2018) | 원시 센서값에서 속도 명령을 바로 내는 분산 정책. 다중 시나리오·다단계 학습, NH-ORCA와 비교 | 정책이 (v, ω)를 직접 내는 구조가 같다. 우리는 다중 로봇이 아니라 단일 로봇의 동역학 변화에 집중한다 |
| Kästner 외, Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems (IROS 2021) | DRL 로컬 플래너를 기존 내비게이션 스택에 넣고 고전 플래너와 같은 틀에서 학습·비교 | DRL 플래너 vs DWA 비교 자체는 이미 있다는 근거. 우리 비교는 질량별(학습·미학습)과 질량 정보 조건별로 나눈다 |
| Xie, Dames, DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles (IEEE T-RO 2023) | LiDAR 이력·보행자 운동 정보·subgoal을 입력으로, VO 기반 보상 항을 쓰는 군중 주행 정책. 재학습 없이 실물 이전 | 경로를 관측으로만 주고 추종 보상을 두지 않는 설계, 시간 초과 페널티를 이 논문 방식으로 따랐다(명세서 4-2절) |

## 4. 적재 질량 인지 주행·DWA

| 논문 | 핵심 | 우리와의 관계/차이 |
| --- | --- | --- |
| Fox, Burgard, Thrun, The Dynamic Window Approach to Collision Avoidance (IEEE Robotics & Automation Magazine 1997) | 짧은 시간 안에 도달 가능한 속도 창(가감속 한계)과 안전하게 설 수 있는 속도만 후보로 두고 목적함수로 (v, ω)를 고른다 | B1·B2의 정의. DWA는 가감속 한계를 아는 값으로 받으므로, 그 값을 빈 차로 고정(B1)하거나 실제 질량에 맞추는(B2) 두 기준선을 둔다 |
| Demir, Sezer, Motion Planning and Control with Randomized Payloads Using Deep Reinforcement Learning (IEEE IRC 2019) | 차동 구동 로봇에서 적재를 무작위화한 DRL 에이전트(바퀴 토크 출력). 학습 때 보지 않은 질량에서 성공률 98.2%, 적재를 모르는 에이전트와 40% 넘게 차이 났다 | **가장 가까운 선행연구.** 적재 무작위화 DRL과 적재 인지 효과는 이미 보였다. 기준선이 DWA가 아니고, 질량 정보 오차·결측과 돌발 장애물 제동은 다루지 않는다 |
| Demir, Sezer, Motion Planning and Control with Randomized Payloads on Real Robot Using Deep Reinforcement Learning (International Journal of Semantic Computing 13(4), 2019) | 위 연구의 저널 확장. TTC 기반 제어기와 비교하고 실물 로봇에 적용 | TTC 기준선과 비교한 점이 우리 안전 지표(최소 TTC)와 닿는다. 우리 기준선은 질량에 맞춘 DWA(B2)다 |
| Prakash 외, Adaptive Model Predictive Control of Wheeled Mobile Robots (arXiv 2201.00863, 2022) | 관성을 모르는 2륜 로봇을 재귀 최소제곱으로 파라미터를 갱신하는 적응형 MPC로 제어. 창고 로봇의 미지 적재를 동기로 든다. 수치 시뮬레이션만 | 모델 기반으로 질량을 추정하는 대안. 장애물 회피와 가감속 한계는 다루지 않는다 |

## 5. 안전: VO·TTC·shielding

| 논문 | 핵심 | 우리와의 관계/차이 |
| --- | --- | --- |
| Fiorini, Shiller, Motion Planning in Dynamic Environments Using Velocity Obstacles (IJRR 17(7), 1998) | 충돌로 이어지는 상대속도 집합(VO)을 정의하고, 회피 속도를 로봇 가속 한계로 허용되는 속도와 교집합해 고른다 | 원 명세 7장 필수인 VO 개념 반응형 회피의 근거. 가속 한계가 질량에 따라 바뀌면 허용 속도 집합도 바뀐다는 점이 우리 문제와 같다 |
| van den Berg 외, Reciprocal n-Body Collision Avoidance (Robotics Research, Springer, 2011) | ORCA. 상호 회피를 저차원 선형계획으로 풀어 다수 로봇의 국소 충돌 회피를 보장 | CADRL·Long 외의 비교 기준. 우리 비교군에는 넣지 않는다 |
| Stern, Shiller, Robot Navigation in Dynamic Environments using Acceleration Obstacles (arXiv 2504.13637, 2025) | VO를 가속도 공간의 Acceleration Obstacle로 확장 | 가감속 한계를 회피 판단에 직접 넣는 최근 모델 기반 접근. 질량 의존 한계와 연결할 수 있는 대안으로 원고 관련연구에 언급한다 |
| Alshiekh 외, Safe Reinforcement Learning via Shielding (AAAI 2018) | 시제 논리 명세로 합성한 shield가 학습 에이전트의 행동을 감시하고 명세를 어길 때만 개입 | Gazebo 평가에서 안전 노드를 켤 때의 개념적 근거. 켜면 개입 횟수를 지표로 기록한다(명세서 4-6절) |
| Ames 외, Control Barrier Function Based Quadratic Programs for Safety Critical Systems (IEEE TAC 62(8), 2017) | 제어 장벽 함수(CBF)로 안전 집합 불변성을 입력 제약으로 바꿔 QP로 푼다. 구동기 한계가 있는 예제 | 안전 필터의 다른 갈래. 우리 연구에서는 쓰지 않고, 안전 필터 없이 정책 자체의 안전성을 본다 |
| He 외, Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion (RSS 2024) | 민첩 정책 + 회복 정책, 학습한 reach-avoid 가치망이 둘 사이 전환을 정한다. Go1에서 최고 3.1 m/s | 남은 시간을 관측에 넣는 설계를 이 논문에서 따왔다(명세서 4-2절). 고속과 안전의 맞교환이라는 문제의식이 같다 |

## 차별점

1. 적재 질량을 무작위화한 DRL 주행과 "적재를 아는 정책이 낫다"는 결과는 차동 구동 로봇에서 이미 보고됐다(Demir·Sezer 2019). 이 연구는 질량 효과를 바퀴 토크 한계에서 생기는 가감속·회전 결합 한계로 모델링하고, 그 효과가 실제로 드러나는 급정지·고가속 구간(돌발 장애물)으로 범위를 좁힌다.
2. 비교 기준을 질량에 맞춘 감속 한계를 쓰는 DWA(B2)로 둔다. "질량을 정확히 아는 기존 방법"을 이기는지가 아니라, 질량 정보가 ±30% 틀리거나 없을 때 RL 정책과 B2가 얼마나 나빠지는지를 같은 조건에서 비교한다. 이를 위해 질량 관측에 앎 여부 플래그를 두고 학습 중 20%를 가린다.
3. 돌발 장애물을 시간이 아니라 경로상 출현 지점에서 나타나게 하고, 그 지점을 가장 무거운 학습 질량으로도 설 수 있는 거리 이후에 둔다. 그래서 모든 비교군이 같은 장애물을 만나고, 충돌은 "설 수 있었음 × 막힘" 집계로 정책의 속도 선택 탓인지 가려낼 수 있다.
4. 알고리즘은 새롭지 않다(PPO, 도메인 무작위화, 질량 관측). 기여는 물류 AMR의 적재 변화를 대상으로 한 문제 설정과 평가 설계이며, 결과는 2D 시뮬레이션과 Gazebo 평가에 한정한다(실물 로봇 없음).

## 참고문헌

- Peng, Andrychowicz, Zaremba, Abbeel (2018). Sim-to-Real Transfer of Robotic Control with Dynamics Randomization. ICRA. https://arxiv.org/abs/1710.06537
- Tobin, Fong, Ray, Schneider, Zaremba, Abbeel (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. IROS. https://arxiv.org/abs/1703.06907
- Yu, Tan, Liu, Turk (2017). Preparing for the Unknown: Learning a Universal Policy with Online System Identification. RSS. https://roboticsproceedings.org/rss13/p48.html
- Kumar, Fu, Pathak, Malik (2021). RMA: Rapid Motor Adaptation for Legged Robots. RSS. https://roboticsproceedings.org/rss17/p011.html
- Lee, Hwangbo, Wellhausen, Koltun, Hutter (2020). Learning Quadrupedal Locomotion over Challenging Terrain. Science Robotics 5(47). https://arxiv.org/abs/2010.11251
- Schulman, Wolski, Dhariwal, Radford, Klimov (2017). Proximal Policy Optimization Algorithms. arXiv:1707.06347. https://arxiv.org/abs/1707.06347
- Chen, Liu, Everett, How (2017). Decentralized Non-communicating Multiagent Collision Avoidance with Deep Reinforcement Learning. ICRA. https://arxiv.org/abs/1609.07845
- Everett, Chen, How (2018). Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning. IROS. https://arxiv.org/abs/1805.01956
- Long, Fan, Liao, Liu, Zhang, Pan (2018). Towards Optimally Decentralized Multi-Robot Collision Avoidance via Deep Reinforcement Learning. ICRA. https://arxiv.org/abs/1709.10082
- Kästner et al. (2021). Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems. IROS. https://arxiv.org/abs/2104.03616
- Xie, Dames (2023). DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles. IEEE Transactions on Robotics. https://arxiv.org/abs/2301.06512
- Fox, Burgard, Thrun (1997). The Dynamic Window Approach to Collision Avoidance. IEEE Robotics & Automation Magazine 4(1). https://doi.org/10.1109/100.580977
- Demir, Sezer (2019). Motion Planning and Control with Randomized Payloads Using Deep Reinforcement Learning. IEEE IRC. https://research.itu.edu.tr/en/publications/motion-planning-and-control-with-randomized-payloads-using-deep-r/
- Demir, Sezer (2019). Motion Planning and Control with Randomized Payloads on Real Robot Using Deep Reinforcement Learning. International Journal of Semantic Computing 13(4). https://research.itu.edu.tr/en/publications/motion-planning-and-control-with-randomized-payloads-on-real-robo/
- Prakash, Perreault, Voth, Zhong (2022). Adaptive Model Predictive Control of Wheeled Mobile Robots. arXiv:2201.00863. https://arxiv.org/abs/2201.00863
- Fiorini, Shiller (1998). Motion Planning in Dynamic Environments Using Velocity Obstacles. International Journal of Robotics Research 17(7). https://doi.org/10.1177/027836499801700706
- van den Berg, Guy, Lin, Manocha (2011). Reciprocal n-Body Collision Avoidance. Robotics Research, Springer. https://www.springerprofessional.de/reciprocal-n-body-collision-avoidance/3604734
- Stern, Shiller (2025). Robot Navigation in Dynamic Environments using Acceleration Obstacles. arXiv:2504.13637. https://arxiv.org/abs/2504.13637
- Alshiekh, Bloem, Ehlers, Könighofer, Niekum, Topcu (2018). Safe Reinforcement Learning via Shielding. AAAI. https://ojs.aaai.org/index.php/AAAI/article/view/11797
- Ames, Xu, Grizzle, Tabuada (2017). Control Barrier Function Based Quadratic Programs for Safety Critical Systems. IEEE Transactions on Automatic Control 62(8). https://arxiv.org/abs/1609.06408
- He, Zhang, Xiao, He, Liu, Shi (2024). Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion. RSS. https://roboticsproceedings.org/rss20/p059.html

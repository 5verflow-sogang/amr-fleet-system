### 1. 미션 소개

AMR(Autonomous Mobile Robot)은 물류센터, 제조공장, 병원 등에서 자재와 물품을 자율적으로 운반하는 이동 로봇 시스템이다.
아마존 물류센터의 Kiva 로봇, 쿠팡 풀필먼트 센터의 AGV 시스템, 병원 내 의약품 배송 로봇처럼 다양한 산업 현장에서 핵심 자동화 인프라로 활용된다.

이 미션에서는 NVIDIA Isaac Sim 또는 Gazebo 시뮬레이터 환경에서 스마트 물류센터를 구축하고, 동적 환경에 대응하는 자율주행 AMR 시스템을 End-to-End로 개발한다.
단순히 기존 라이브러리를 조합하는 수준을 넘어, 로봇공학의 핵심 알고리즘을 직접 구현하고 원리를 이해하는 것을 목표로 한다.

이동 로봇의 키네마틱스부터 센서 데이터 처리, SLAM, 경로 계획, 모션 제어, 딥러닝 기반 객체 인식, 작업 수행까지 전체 파이프라인을 구축하며, 다중 로봇 협업을 위한 Fleet Management 시스템까지 설계한다.
또한, Docker 기반 개발 환경과 Git Flow 전략을 적용하여 실무 수준의 개발 프로세스를 경험한다.
시뮬레이션 기반 개발 방법론을 통해 센서 노이즈 모델링, Domain Randomization, Sim-to-Real Gap 분석 등 실제 로봇 배포를 고려한 개발 역량을 습득한다.

### 2. 최종 결과물

다음 4가지 핵심 시스템이 정상 동작하는 시뮬레이션 기반 지능형 물류 AMR 시스템을 완성하고, GitHub 레포지토리(Repository) URL을 제출한다.

1. **시뮬레이션 환경 및 로봇 시스템 구축**
   - 입력: 물류센터 설계 파라미터, 로봇 스펙 정의
   - 출력: 현실적인 물리 시뮬레이션이 적용된 물류센터 환경, 센서 노이즈가 모델링된 AMR 로봇, Docker 기반 ROS2 소프트웨어 스택
2. **자율 매핑 및 정밀 위치 추정 시스템**
   - 입력: LiDAR, IMU, Wheel Encoder 데이터
   - 출력: 고정밀 2D 맵 생성, EKF 기반 센서 퓨전 위치 추정, 위치 추정 오차 분석 리포트
3. **AI 기반 인지 및 동적 환경 대응 자율주행 시스템**
   - 입력: 목표 지점, 실시간 센서 데이터, 카메라 영상
   - 출력: 딥러닝 기반 객체 인식 및 3D 위치 추정, 최적 경로 생성 및 추종, 동적 장애물 예측 및 회피, 긴급 상황 대응
4. **다중 AMR 협업 운용 시스템**
   - 입력: 다수의 물품 배송 요청
   - 출력: 5대 이상 AMR 동시 운용, 작업 최적 할당, 교착 방지, 실시간 모니터링 대시보드

제출물은 GitHub 레포지토리 URL이며, 다음 내용이 포함되어야 한다.

- 전체 소스 코드 및 설정 파일
- Docker 환경 구성 파일 (docker-compose.yml)
- 시스템 아키텍처 설계 문서
- 핵심 알고리즘 구현 설명 문서
- 성능 측정 및 분석 리포트
- 시뮬레이션 환경 재현을 위한 설치/실행 가이드

### 3. 과제 목표

이 과제를 마친 후, 학습자는 아래를 스스로 설명할 수 있어야 한다.

- 차동 구동 로봇의 순기구학 수식과 오도메트리 누적 오차의 원인 및 보완 방법을 설명할 수 있다.
- EKF 기반 센서 퓨전의 예측-업데이트 단계와 각 센서별 공분산 설정 근거를 설명할 수 있다.
- A\*, DWA 등 경로 계획 알고리즘의 동작 원리와 환경에 따른 파라미터 튜닝 방법을 이해한다.
- 딥러닝 모델의 2D 인식 결과를 3D 공간 좌표로 변환하는 원리(Pinhole Camera Model)를 설명할 수 있다.
- Behavior Tree와 Finite State Machine의 차이점과 각각의 장단점을 비교할 수 있다.
- 다중 로봇 시스템에서 발생하는 교착 상황의 유형과 해결 전략을 설명할 수 있다.

### 4. 기능 요구 사항

다음 요구사항을 모두 만족해야 한다.

1. **개발 환경 및 시스템 인프라**
   - Docker 환경 구성
     - 모든 개발 환경은 Docker Container로 구성되어야 하며 docker-compose up 명령으로 실행 가능해야 한다.
     - GPU 가속이 필요한 경우 nvidia-container-toolkit을 활용한다.
     - ros:humble 또는 ros:iron 공식 이미지를 베이스로 Dockerfile을 작성한다.
   - 형상 관리
     - Git Flow 전략(Main, Develop, Feature)을 사용하여 형상 관리를 수행해야 한다.
     - 커밋 메시지는 conventional commits 형식을 따른다.
   - 시뮬레이터 환경 설정
     - NVIDIA Isaac Sim 2023.1 이상 또는 Gazebo Fortress/Garden 버전을 사용한다.
     - 물류센터 환경은 최소 60m x 40m 크기로 구성하며, 선반 구역, 입고/출고 구역, 충전 스테이션, 통로를 포함한다.
     - 바닥 마찰 계수, 조명 조건 등 물리 파라미터를 현실적으로 설정한다.
     - 정적 장애물(선반, 벽, 기둥)과 동적 장애물(이동하는 작업자, 지게차, 다른 로봇)을 포함한다.
     - 동적 장애물은 최소 5개 이상이며, 다양한 속도(0.3~1.5m/s)와 이동 패턴(직선, 곡선, 무작위)을 가진다.
   - 로봇 모델링
     - 차동 구동 방식의 AMR 로봇 모델을 URDF 또는 SDF로 직접 작성한다.
     - 로봇 URDF 모델은 xacro로 모듈화되어야 하며, 실제 센서 위치와 TF 트리가 정확히 일치해야 한다.
     - 로봇 크기는 실제 물류 AMR 규격(약 600mm x 400mm x 300mm)을 참고한다.
     - 최대 속도 2.0m/s, 최대 가속도 1.0m/s², 최대 회전 속도 1.5rad/s의 동역학 제한을 적용한다.
     - 적재 용량에 따른 질량 변화가 로봇 동역학에 반영되어야 한다.
   - 센서 시스템 구성
     - 2D LiDAR: 360도 스캔, 최대 거리 25m, 해상도 0.5도, 10Hz 이상. 가우시안 노이즈 모델(σ=0.03m)을 적용한다.
     - Depth Camera: 해상도 640x480 이상, FOV 87도, 최대 거리 10m, 15Hz 이상. 깊이 측정 노이즈를 적용한다.
     - RGB Camera: 해상도 640x480 이상, 30Hz. 객체 인식용으로 사용한다.
     - IMU: 가속도계 및 자이로스코프, 100Hz 이상. 바이어스와 노이즈 모델을 적용한다.
     - Wheel Encoder: 틱 분해능 4096 이상, 슬립 노이즈를 적용한다.
     - 센서 캘리브레이션 절차를 문서화하고, 외부 파라미터(extrinsic) 설정 파일을 제공한다.
2. **로봇 기초 시스템 구현**
   - 키네마틱스 및 오도메트리
     - 차동 구동 로봇의 순기구학(Forward Kinematics)을 직접 구현한다.
     - Wheel Encoder 기반 오도메트리를 구현하고, 누적 오차 특성을 분석한다.
     - 오도메트리 드리프트 측정 실험을 수행하고 결과를 리포트에 포함한다.
   - 센서 데이터 처리
     - LiDAR 포인트 클라우드 필터링(거리 필터, 각도 필터, 아웃라이어 제거)을 구현한다.
     - Depth Camera의 포인트 클라우드 변환 및 다운샘플링을 구현한다.
     - IMU 데이터 필터링(저역 통과 필터) 및 바이어스 보정을 구현한다.
   - TF 시스템 구성
     - 로봇의 전체 TF 트리를 설계하고 구현한다.
     - map → odom → base_link → 각 센서 프레임의 관계를 명확히 정의한다.
     - TF 트리 구조를 문서화하고 시각화 자료를 포함한다.
3. **위치 추정 시스템**
   - SLAM 구현
     - 2D LiDAR 기반 SLAM을 구현한다. slam_toolbox 또는 cartographer를 기반으로 하되, 주요 파라미터의 의미와 튜닝 근거를 문서화한다.
     - 또는 Scan Matching 기반 SLAM의 핵심 로직(ICP 또는 NDT)을 직접 구현한다.
     - Loop Closure 탐지 및 그래프 최적화 과정을 이해하고 설명할 수 있어야 한다.
     - 생성된 맵은 Occupancy Grid 형태로 저장하며, 해상도는 0.05m 이하로 설정한다.
     - 맵 품질 평가 지표(맵 일관성, 구조물 정확도)를 정의하고 측정한다.
   - 위치 추정
     - AMCL을 적용하고, 파티클 수, 리샘플링 파라미터 등을 환경에 맞게 튜닝한다.
     - 로봇을 임의 위치로 납치(Kidnapped)했을 때, 제자리 회전 등을 통해 스스로 위치를 복구해야 한다.
     - EKF 기반 센서 퓨전(robot_localization 또는 직접 구현)을 적용하여 Wheel Odometry, IMU, AMCL 출력을 융합한다.
     - 센서 퓨전의 각 입력에 대한 공분산 행렬을 적절히 설정하고 근거를 문서화한다.
     - 위치 추정 오차는 정지 상태에서 3cm, 직선 주행 시 5cm, 회전 주행 시 8cm 이내를 만족해야 한다.
     - 위치 추정 정확도를 Ground Truth와 비교하여 RMSE, Maximum Error를 측정하고 리포트에 포함한다.
4. **경로 계획 시스템**
   - Global Planner
     - A\* 알고리즘을 직접 구현하고, NavFn 또는 Smac Planner와 성능을 비교한다.
     - 비교 지표는 경로 길이, 계획 소요 시간, 장애물 여유 거리를 포함한다.
     - 경로 평활화(Path Smoothing)를 적용하여 로봇이 추종 가능한 경로를 생성한다.
   - Local Planner
     - DWA(Dynamic Window Approach) 알고리즘의 핵심 로직을 직접 구현한다.
     - 속도 샘플링, 궤적 시뮬레이션, 비용 함수 계산, 최적 속도 선택 과정을 구현한다.
     - TEB Planner와 성능을 비교하고, 각 알고리즘의 장단점을 분석한다.
   - Costmap 구성
     - Static Layer, Obstacle Layer, Inflation Layer를 포함한 Costmap을 구성한다.
     - Inflation 반경, 비용 스케일링 파라미터를 환경에 맞게 튜닝한다.
     - 로봇 폭 +20cm 이내의 좁은 통로를 충돌 없이 주행할 수 있도록 파라미터를 조정한다.
     - 동적 장애물 반영을 위한 센서 업데이트 주기 및 장애물 유지 시간을 설정한다.
   - 경로 계획 성능
     - 임의의 출발지-목적지 쌍 50개에 대해 경로 계획 성공률 98% 이상을 달성한다.
     - 경로 재계획이 500ms 이내에 완료되어야 한다.
     - 계획된 경로의 실제 주행 시간 대비 예측 시간 오차가 15% 이내여야 한다.
5. **모션 제어 시스템**
   - 경로 추종 제어
     - Pure Pursuit 또는 Stanley Controller를 직접 구현한다.
     - Look-ahead 거리, 게인 파라미터를 속도에 따라 적응적으로 조정한다.
     - 경로 추종 오차(Cross Track Error)가 직선 구간에서 5cm, 곡선 구간에서 10cm 이내를 유지한다.
   - 속도 제어
     - PID 기반 속도 제어기를 구현하고, 게인 튜닝 과정을 문서화한다.
     - 속도 프로파일링(사다리꼴 또는 S-curve)을 적용하여 급격한 가감속을 방지한다.
     - 최대 저크(Jerk) 제한을 적용하여 적재물 안정성을 확보한다.
6. **AI 기반 인지 시스템**
   - 객체 인식
     - YOLOv8(또는 동급 모델)을 사용하여 화물, 사람, 표지판 등 3종 이상의 객체를 인식해야 한다.
     - 인식된 객체 정보를 커스텀 메시지 타입으로 발행하고, 이를 RViz2에 마커(Marker)로 시각화해야 한다.
     - 추론 속도는 CPU 기준 10 FPS 이상(또는 GPU 가속 시 30 FPS 이상) 유지되어야 한다.
   - 3D 좌표 변환
     - 카메라의 2D 픽셀 좌표를 Depth 정보와 결합하여 3D 공간 좌표(Map frame)로 변환한다.
     - Pinhole Camera Model 기반 좌표 변환 원리를 이해하고 구현한다.
     - 인식된 객체의 3D 위치가 지도상에 마커로 표시되어야 한다.
7. **동적 환경 대응**
   - 동적 장애물 감지 및 추적
     - LiDAR 데이터에서 동적 장애물을 분류하는 알고리즘을 구현한다.
     - 칼만 필터 또는 확장 칼만 필터 기반 객체 추적을 구현한다.
     - 추적 객체의 속도, 이동 방향을 추정하고 신뢰도를 계산한다.
   - 충돌 예측 및 회피
     - 추적된 장애물의 미래 위치를 예측하고, 로봇 경로와의 충돌 가능성을 계산한다.
     - Time-To-Collision(TTC) 지표를 계산하고, 임계값 이하 시 회피 기동을 수행한다.
     - 1.0m/s 속도로 움직이는 동적 장애물을 인식하고 경로를 재계획(Replanning)하여 회피해야 한다.
     - Velocity Obstacle 또는 ORCA 개념을 적용한 반응형 회피를 구현한다.
   - 안전 시스템
     - 긴급 정지 기능을 구현한다. 장애물이 안전 거리(0.3m) 이내로 접근 시 즉시 정지한다.
     - Safety Zone(Warning/Critical)을 정의하고, 단계별 대응 로직을 구현한다.
     - 비상 정지(E-Stop) 버튼을 누르면 로봇이 즉시 물리적으로 정지해야 한다.
     - 센서 고장 감지 로직을 구현하고, 고장 시 안전 정지 또는 저속 운행으로 전환한다.
   - 동적 환경 대응 성능
     - 동적 장애물 회피 테스트 시나리오 30회 수행 시 충돌 0건을 달성한다.
     - 회피 기동 시 경로 이탈이 1m 이내여야 한다.
     - 회피 후 원래 경로로 복귀하는 시간이 5초 이내여야 한다.
8. **작업 수행 시스템**
   - Behavior Tree 설계
     - BehaviorTree.CPP를 활용하여 작업 수행 로직을 구현한다.
     - '대기-이동-인식-작업-복귀' 시나리오를 구현해야 한다.
     - 최소 15개 이상의 노드(Action, Condition, Control, Decorator)를 활용한다.
     - 재사용 가능한 서브트리를 설계하고, 새로운 작업 추가가 용이한 구조를 만든다.
     - 주행 불가, 인식 실패 등 3가지 이상의 에러 상황에 대한 자동 복구(Recovery) 로직이 포함되어야 한다.
     - Behavior Tree 구조를 시각화(Groot)하고 문서화한다.
   - 도킹 시스템
     - 선반/스테이션 앞 정밀 도킹 알고리즘을 구현한다.
     - 도킹 마커(ArUco 또는 가상 마커) 인식 기반 정밀 접근을 구현한다.
     - 도킹 정밀도는 위치 오차 2cm, 각도 오차 1도 이내를 만족한다.
     - 도킹 실패 시 재시도 로직을 포함하며, 최대 3회 실패 시 에러 보고 및 대체 작업을 수행한다.
     - 가상 물품 및 적재/하역 이벤트 참고값
       | **물품 유형** | **질량** | **크기 (cm)** | **적재 시간** | **비고** |
       | ------------------- | -------------- | ------------------- | ------------------- | --------------------------------------- |
       | 소형 박스 | 2 kg | 30 x 20 x 15 | 5초 | 전자부품, 소형 패키지 |
       | 중형 박스 | 10 kg | 50 x 40 x 30 | 10초 | 의류, 생활용품 |
       | 대형 박스 | 25 kg | 60 x 50 x 40 | 15초 | 가전, 산업자재 (적재 시 질량 변화 반영) |
   - 작업 관리
     - 작업 명령은 JSON 형식의 Task Description으로 수신한다.
     - 작업 상태(Pending, InProgress, Completed, Failed)를 관리하고 이벤트로 발행한다.
     - 작업 우선순위, 마감 시간을 고려한 스케줄링을 지원한다.
     - 단일 작업 성공률 97% 이상을 달성한다.
     - 작업 로그(시작 시간, 종료 시간, 이동 거리, 소요 시간, 결과)를 기록하고 분석 가능한 형태로 저장한다.
9. **Fleet Management 시스템**
   - 다중 로봇 환경 구성
     - 최소 5대의 AMR을 동시에 운용한다.
     - 각 로봇은 독립된 네임스페이스를 가지며, TF 프레임 충돌이 없어야 한다.
     - 로봇 간 통신 지연(최대 100ms)을 시뮬레이션에 반영한다.
   - 작업 할당
     - 중앙 집중식 작업 할당 시스템을 구현한다.
     - 할당 알고리즘으로 최소 거리 기반, 부하 균형, 또는 Hungarian Algorithm을 구현한다.
     - 작업 할당 최적성 지표(총 이동 거리, 작업 완료 시간)를 측정하고 분석한다.
   - Traffic Management
     - 경로 충돌 예측 및 해소 알고리즘을 구현한다.
     - 교차로/병목 구간에서의 우선순위 결정 로직을 구현한다.
     - 교착(Deadlock) 탐지 알고리즘을 구현하고, 탐지 시 해소 전략을 적용한다.
     - 교착 해소 전략 최소 2가지(예: 우선순위 기반 양보, 대체 경로 탐색)를 구현한다.
   - 모니터링 시스템
     - Foxglove Studio 또는 웹 기반 모니터링 대시보드를 구현한다.
     - 실시간 로봇 위치, 상태, 배터리 잔량, 현재 작업을 시각화한다.
     - 시스템 전체 KPI(작업 처리량, 평균 작업 시간, 로봇 가동률)를 표시한다.
     - 이상 상황(긴급 정지, 작업 실패, 교착) 발생 시 알림 기능을 구현한다.
10. **시스템 통합 및 품질**
    - 성능 요구사항
      - 시스템 평균 응답 시간(명령 수신 ~ 로봇 반응)이 200ms 이내여야 한다.
      - 5대 로봇 동시 운용 시 CPU 사용률이 80% 이하여야 한다.
      - 연속 4시간 운용 테스트에서 시스템 안정성을 검증한다.
    - 테스트 및 검증
      - 단위 테스트를 작성하고, 주요 모듈의 테스트 커버리지 70% 이상을 달성한다.
      - 통합 테스트 시나리오를 최소 10개 작성하고 자동화한다.
      - 성능 테스트 결과를 정량적 지표와 함께 리포트로 작성한다.
      - 성능 지표 측정 절차 표준화
        | **지표** | **산출 방법** | **로그 포맷 예시** | **비고** |
        | --------------- | ---------------------------------------------------- | -------------------------------------------- | --------------------------- |
        | 위치 추정 RMSE | Ground Truth 좌표 - EKF 추정 좌표의 제곱 평균 제곱근 | [timestamp, gt_x, gt_y, est_x, est_y, error] | 최소 100개 시점 샘플링 |
        | 경로 추종 CTE | 계획 경로와 실제 주행 궤적 간 수직 거리 평균 | [timestamp, planned_x/y, actual_x/y, cte] | 직선/곡선 구간 분리 측정 |
        | 응답 시간 | 작업 명령 발행 timestamp - 로봇 첫 움직임 timestamp | [cmd_time, response_time, latency_ms] | 50회 이상 측정 후 평균/최대 |
        | 테스트 커버리지 | pytest --cov 또는 colcon test --coverage 출력 | coverage report 텍스트/HTML 출력 | 주요 모듈별 분리 보고 |
    - 문서화
      - README.md에 프로젝트 개요, 아키텍처, 설치 방법, 실행 방법을 작성한다.
      - 시스템 아키텍처 문서에 컴포넌트 다이어그램, 시퀀스 다이어그램을 포함한다.
      - 핵심 알고리즘(DWA, EKF, SLAM 파라미터 등) 구현 및 튜닝 과정을 문서화한다.
      - API 문서를 작성하여 각 노드의 토픽, 서비스, 액션 인터페이스를 명세한다.

- 구현 지침

  아래는 문제 해결에 도움이 될 수 있는 검색 키워드와 권장 순서다. 정답이 아니므로 자유롭게 접근해도 된다.

  개발 환경 구성
  - Docker 환경 설정이 어렵다면 ros:humble 공식 이미지를 베이스로 Dockerfile을 작성하는 방법부터 찾아본다.
  - docker-compose, nvidia-container-toolkit 키워드로 검색하여 GPU 가속 설정을 확인한다.

  시뮬레이션 및 로봇 모델
  - Gazebo Fortress(Ignition)와 Isaac Sim 중 팀의 하드웨어 사양(GPU 유무)에 맞춰 시뮬레이터를 선정한다.
  - URDF 작성 시 link와 joint의 관계를 먼저 종이에 그려보고 시작한다. urdf tutorial, xacro 키워드를 참고한다.

  자율주행 및 SLAM
  - 지도가 찌그러지거나 매칭이 안 된다면 slam_toolbox의 loop_match_minimum_chain_size, correlation_search_space 파라미터를 조정해본다.
  - 좁은 길 주행이 어렵다면 costmap_2d, inflation_radius, cost_scaling_factor 키워드로 검색하여 로봇 주변의 비용 지도를 분석한다.

  AI 모델 연동
  - YOLO 모델을 ROS2 노드로 만들 때는 ultralytics, cv_bridge, ros2 topic pub을 활용하여 이미지가 정상적으로 수신되는지 먼저 테스트한다.
  - 2D 픽셀 좌표를 3D 공간 좌표로 변환하려면 pinhole camera model, depth camera projection 원리를 찾아본다.

  구현 순서 권장
  1. Docker 개발 환경 구축 및 Git 리포지토리 생성
  2. 시뮬레이터(Gazebo/Isaac) 월드 로드 및 로봇 스폰(URDF)
  3. 센서 데이터(LiDAR, Camera, IMU) 토픽 확인 및 TF 트리 검증
  4. SLAM 매핑 및 지도 저장
  5. EKF 센서 퓨전 구성 및 위치 추정 테스트
  6. Nav2 설정 및 기본 주행 테스트
  7. DWA, A\* 핵심 로직 직접 구현 및 비교
  8. YOLO 객체 인식 노드 개발 및 3D 변환 연동
  9. 동적 장애물 추적 및 예측 기반 회피 구현
  10. Behavior Tree 기반 시나리오 통합 및 예외 처리
  11. Fleet Management 시스템 구현
  12. 대시보드 구축 및 최종 최적화
  13. 테스트 자동화 및 문서화

### 5. 보너스 과제 (선택)

1. 강화학습 기반 내비게이션
   - Isaac Sim의 강화학습 환경을 활용하여 PPO/SAC 기반 내비게이션 정책을 학습시킨다.
   - 학습된 정책과 기존 DWA의 성능을 정량적으로 비교한다.
   - Domain Randomization을 적용하여 일반화 성능을 향상시킨다.
   - 시뮬레이션 환경에서의 로봇 강화학습 파이프라인 전체를 경험한다.
2. 3D LiDAR 기반 SLAM
   - 3D LiDAR 센서를 추가하고 3D SLAM(LIO-SAM 등)을 적용한다.
   - 2D SLAM 대비 맵 품질 및 위치 추정 정확도를 비교 분석한다.
   - 3D 포인트 클라우드 처리 및 최적화 기법을 학습한다.
3. 생성형 AI 음성 명령 연동
   - STT(Speech to Text)와 LLM을 연동하여 "A구역으로 가서 박스 가져와" 같은 자연어 명령을 로봇의 작업 시나리오로 변환하여 실행한다.
   - 최신 AI 트렌드인 로보틱스와 LLM의 융합(VLA) 가능성을 체험할 수 있다.
4. 분산 Fleet Management
   - 중앙 집중식에서 분산 협업 방식으로 Fleet Management를 확장한다.
   - 로봇 간 직접 통신을 통한 협상 기반 작업 할당을 구현한다.
   - 중앙 서버 장애 시에도 로봇들이 자율적으로 동작하는 폴백 로직을 구현한다.

---

### 6. 개발 환경

- Ubuntu 22.04 LTS, ROS2 Humble Hawksbill 버전 사용
- 모든 개발 환경은 Docker Container로 구성
- Python 3.10 이상 또는 C++ 17 이상
- 시뮬레이터 Gazebo Fortress 또는 NVIDIA Isaac Sim 2023.1 이상

### 7. 제약 사항

- 소프트웨어 스택
  - ROS2 Humble 또는 Iron 버전을 사용한다.
  - Navigation은 Nav2를 기반으로 하되, DWA Local Planner와 A\* Global Planner의 핵심 로직은 직접 구현하여 원리를 이해한다.
  - 객체 인식은 YOLOv8 또는 동급 모델을 사용한다.
  - 외부 상용 솔루션은 사용하지 않는다.
- 구현 범위
  - 로봇 팔을 활용한 물리적 물품 조작은 이 미션 범위에 포함하지 않는다.
  - 물품 픽업/배송은 도킹 완료 후 가상의 적재/하역 이벤트로 대체한다.
  - LLM(Large Language Model) 기반의 자연어 명령 처리는 구현하지 않는다. (보너스 과제 제외)
  - 3D SLAM은 보너스 과제로만 다루며, 필수 요구사항은 2D SLAM이다.
- 프로젝트 구조
  - 모든 소스 코드는 src 폴더 내에 패키지 단위로 구분되어야 한다.
  - colcon build 명령 수행 시 에러나 경고(Warning)가 발생하지 않아야 한다.
  - 센서 노이즈 모델을 반드시 적용하여 현실적인 환경을 구성한다.

### 8. 결과 예시

아래는 정답이 아니라 참고 예시다. 실제 문구, 디자인, 구현 방식은 달라도 된다.

- 시뮬레이션 환경 예시

  ```bash
  [물류센터 시뮬레이션 환경 레이아웃 - 60m x 40m]
  ┌──────────────────────────────────────────────────────────────┐
  │  [입고구역]                                     [출고구역]    │
  │  ┌──────┐                                         ┌──────┐   │
  │  │Dock-1│    ══════════════════════════════════   │Dock-A│   │
  │  │Dock-2│         (Main Corridor                  │Dock-B│   │
  │  └──────┘    ══════════════════════════════════   └──────┘   │
  │                                                              │
  │  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐     │
  │  │Rack │ │Rack │ │Rack │ │Rack │ │Rack │ │Rack │ │Rack │     │
  │  │ A-1 │ │ A-2 │ │ A-3 │ │ A-4 │ │ A-5 │ │ A-6 │ │ A-7 │     │
  │  └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘     │
  │     [Worker]        [AMR-01]                    [Forklift    │
  │  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐     │
  │  │Rack │ │Rack │ │Rack │ │Rack │ │Rack │ │Rack │ │Rack │     │
  │  │ B-1 │ │ B-2 │ │ B-3 │ │ B-4 │ │ B-5 │ │ B-6 │ │ B-7 │     │
  │  └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘     │
  │              [AMR-02]              [AMR-03]                  │
  │  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐     │
  │  │Rack │ │Rack │ │Rack │ │Rack │ │Rack │ │Rack │ │Rack │     │
  │  │ C-1 │ │ C-2 │ │ C-3 │ │ C-4 │ │ C-5 │ │ C-6 │ │ C-7 │     │
  │  └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘     │
  │         [AMR-04]                         [AMR-05]            │
  │  ┌──────────────┐                        ┌──────────────┐    │
  │  │Charging Zone │                        │ Waiting Zone │    │
  │  │ [C1][C2][C3] │                        │              │    │
  │  └──────────────┘                        └──────────────┘    │
  └──────────────────────────────────────────────────────────────┘
  ```

- TF 트리 구조 예시

  ```bash
  map
  └── odom (published by robot_localization)
      └── base_footprint
          └── base_link
              ├── lidar_link (x: 0.15, y: 0, z: 0.2)
              ├── camera_link (x: 0.18, y: 0, z: 0.25)
              │   └── camera_optical_frame
              ├── imu_link (x: 0, y: 0, z: 0.1)
              ├── left_wheel_link
              └── right_wheel_link
  ```

- Vision AI 객체 인식 결과 예시

  ```bash
  터미널 출력:
  [INFO] [yolo_node]: Detected: Box (id=0), Confidence: 0.92
  [INFO] [yolo_node]: Detected: Person (id=1), Confidence: 0.88
  [INFO] [vision_processor]: Transform 2D(320, 240) to 3D(map frame: x=12.5, y=3.2, z=0.2)

  RViz2 화면:
  - 로봇 전방 카메라 화각 내에 'Box'가 감지되자, 지도상 해당 위치에 빨간색 육면체 마커가 생성
  - 마커 위에 텍스트로 "Class: Box, Conf: 0.92, Dist: 1.5m" 표시
  ```

- EKF 센서 퓨전 설정 예시

  ```yaml
  # robot_localization EKF 설정
  ekf_filter_node:
    ros__parameters:
      frequency: 50.0
      two_d_mode: true

      odom0: /wheel_odom
      odom0_config: [
          false,
          false,
          false, # x, y, z
          false,
          false,
          false, # roll, pitch, yaw
          true,
          true,
          false, # vx, vy, vz
          false,
          false,
          true, # vroll, vpitch, vyaw
          false,
          false,
          false,
        ] # ax, ay, az
      odom0_differential: false

      imu0: /imu/data
      imu0_config: [
          false,
          false,
          false,
          false,
          false,
          true, # yaw from IMU
          false,
          false,
          false,
          false,
          false,
          true, # vyaw from IMU
          true,
          true,
          false,
        ] # ax, ay from IMU
      imu0_differential: false

      pose0: /amcl_pose
      pose0_config: [
          true,
          true,
          false, # x, y from AMCL
          false,
          false,
          true, # yaw from AMCL
          false,
          false,
          false,
          false,
          false,
          false,
          false,
          false,
          false,
        ]
  ```

- DWA 알고리즘 핵심 로직 예시

  ```python
  # DWA 알고리즘 개념적 구조 (참고용)
  def dwa_planning(current_state, goal, obstacles):
      # 1. Dynamic Window 계산
      Vs = calculate_search_space(current_state, robot_config)

      # 2. 속도 샘플링
      velocity_samples = sample_velocities(Vs, resolution)

      best_score = -float('inf')
      best_velocity = None

      for v, w in velocity_samples:
          # 3. 궤적 시뮬레이션
          trajectory = simulate_trajectory(current_state, v, w, dt, horizon)

          # 4. 충돌 검사
          if check_collision(trajectory, obstacles):
              continue

          # 5. 비용 함수 계산
          heading_score = calc_heading_score(trajectory, goal)
          clearance_score = calc_clearance_score(trajectory, obstacles)
          velocity_score = calc_velocity_score(v)

          total_score = (alpha * heading_score +
                        beta * clearance_score +
                        gamma * velocity_score)

          if total_score > best_score:
              best_score = total_score
              best_velocity = (v, w)

      return best_velocity
  ```

- Docker Compose 구성 예시

  ```yaml
  # docker-compose.yml
  version: "3.8"
  services:
    ros2-base:
      build:
        context: .
        dockerfile: Dockerfile
      image: amr-fleet-system:latest
      container_name: amr_simulation
      environment:
        - DISPLAY=${DISPLAY}
        - ROS_DOMAIN_ID=0
      volumes:
        - /tmp/.X11-unix:/tmp/.X11-unix:rw
        - ./src:/ros2_ws/src
        - ./config:/ros2_ws/config
      network_mode: host
      privileged: true
      deploy:
        resources:
          reservations:
            devices:
              - driver: nvidia
                count: all
                capabilities: [gpu]
  ```

### 9. 동료 평가 질문 예시

기능 동작 검증

- [ ] README의 설치 및 실행 가이드대로 docker-compose up 명령으로 환경 구성이 완료되는가?
- [ ] 시뮬레이션 환경이 정상 로드되고, 센서 노이즈가 적용된 것을 확인할 수 있는가?
- [ ] SLAM을 통해 생성된 맵이 실제 환경과 일치하며, 맵 해상도가 0.05m 이하인가?
- [ ] EKF 센서 퓨전이 동작하고, 위치 추정 오차가 요구사항을 만족하는가?? (정지: 3cm, 직선: 5cm, 회전: 8cm 이내)
- [ ] YOLO 객체 인식이 동작하고, 인식된 객체가 3D 마커로 RViz2에 시각화되는가?
- [ ] 직접 구현한 DWA가 정상 동작하고, 경로 추종이 안정적인가?(CTE 직선 5cm, 곡선 10cm 이내)
- [ ] 동적 장애물을 추적하고 예측 기반 회피가 수행되는가?(30회 테스트 충돌 0건 확인)
- [ ] Kidnapped Robot 상황에서 스스로 위치를 복구하는가?
- [ ] 도킹 정밀도가 요구사항을 만족하는가?(위치 오차 2cm, 각도 오차 1도 이내)
- [ ] 5대 로봇 동시 운용 시 교착 없이 작업이 수행되는가?
- [ ] 모니터링 대시보드에서 실시간 상태와 KPI를 확인할 수 있는가?

코드 구조 및 설계

- [ ] ROS2 패키지가 기능별로 명확히 분리되고, 각 패키지의 책임이 단일한가?
- [ ] 핵심 알고리즘(DWA, A\*, EKF 등)이 독립 모듈로 구현되어 테스트 가능한가?
- [ ] 설정값이 YAML 파일로 외부화되고, 파라미터 변경 시 재빌드가 필요 없는가?
- [ ] TF 트리가 문서화되어 있고, 좌표계 간 관계가 명확한가? (확인: ros2 run tf2_tools view_frames)
- [ ] Behavior Tree가 모듈화되어 있는가? 새로운 BT 노드를 추가하여 작업을 확장하는 데 기존 코드 수정이 3개 파일 이내인가?
- [ ] 에러 처리 및 복구 로직이 체계적으로 구현되어 있는가?
- [ ] 테스트 코드가 작성되어 있고, 주요 모듈의 커버리지가 70% 이상인가?

핵심 기술 원리 적용

- [ ] 차동 구동 로봇의 키네마틱스가 올바르게 구현되고 문서화되었는가?
- [ ] EKF의 예측/업데이트 단계가 명확히 구현되고, 공분산 설정 근거가 문서화되었는가?
- [ ] DWA의 속도 샘플링, 궤적 시뮬레이션, 비용 함수가 올바르게 구현되었는가?
- [ ] Costmap 레이어 구성이 적절하고, 동적 장애물 반영이 실시간으로 동작하는가?
- [ ] 칼만 필터 기반 객체 추적이 구현되고, 예측 정확도가 적절한가?
- [ ] Pinhole Camera Model 기반 2D→3D 좌표 변환이 올바르게 구현되었는가?
- [ ] 교착 탐지 및 해소 알고리즘이 구현되고, 실제 동작하는가?

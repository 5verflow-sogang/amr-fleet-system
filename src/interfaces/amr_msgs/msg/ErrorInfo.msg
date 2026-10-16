# 오류 1건. 자유 문자열 대신 코드로 분류해 BT 복구 분기와 하네스 집계에 쓴다.
# 코드 대역 (초안): 100 할당 / 200 주행 / 300 교통 / 400 통신 / 500 작업 수행 / 900 도킹
# 900번대는 Dock.action 결과의 error_code와 같은 값이다.

uint16 NONE=0

# 100번대: 할당
uint16 ASSIGN_BATTERY_LOW=101        # 배터리 부족으로 거부
uint16 ASSIGN_ALREADY_BUSY=102       # 이미 작업 수행 중
uint16 ASSIGN_CANCELED=103          # 서버가 취소함 (TaskAssignment.ACTION_CANCEL). FAILED와 함께 보내며, 서버는 다시 할당하지 않는다
uint16 ASSIGN_TASK_TAKEN=104        # 다른 로봇이 먼저 수락한 작업 (하이브리드 중복 정리, architecture.md 5-2절). 거부 Ack나 FAILED와 함께 보내며, 재시도로 세지 않는다

# 200번대: 주행
uint16 NAV_NO_PATH=201               # 경로 없음
uint16 NAV_TIMEOUT=202               # 목표 도달 시간 초과
uint16 LOCALIZATION_LOST=203         # 위치를 잃음 (위치 복구 실패, 또는 복구를 기다리다 시간 초과)

# 300번대: 교통
uint16 TRAFFIC_LEASE_TIMEOUT=301     # 구역 허가 시간 초과
uint16 TRAFFIC_REVOKED=302           # 교착 해소로 허가 회수
uint16 TRAFFIC_BACKING_FAILED=303    # 교착 해소 후진이 안전 한도를 넘음 (수동 확인 또는 리셋 필요)

# 400번대: 통신
uint16 COMM_HEARTBEAT_LOST=401       # 서버 heartbeat 끊김
uint16 COMM_ACK_RETRY_EXCEEDED=402   # Ack 재전송 한도 초과

# 500번대: 작업 수행 (BT)
uint16 TASK_TIMEOUT=501              # 작업 전체 시간 상한 초과 (멈춤·재개를 되풀이해 끝나지 않는 경우 포함)
uint16 CHARGE_TIMEOUT=502            # 충전이 제한 시간 안에 목표량까지 안 참

# 오류 코드. 오류가 없으면 NONE
uint16 code

# 사람이 읽는 설명. 예: "docking failed 3 times"
string message

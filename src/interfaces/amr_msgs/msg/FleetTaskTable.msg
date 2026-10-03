# 서버가 전체 작업 목록과 할당 현황을 주기적으로 방송한다.
# 토픽: /fleet/task_table
# 서버가 죽은 뒤 로봇끼리 경매하려면 남은 작업을 미리 알고 있어야 한다. 대시보드 작업 목록도 이걸 쓴다.

# stamp: 보낸 시각
std_msgs/Header header

# 서버 이름
string server_id

# 서버 시작 번호 (FleetHeartbeat.epoch와 같은 값)
uint32 epoch

# epoch 안에서 1씩 늘어나는 번호
uint32 seq

# 작업별 현황
TaskRecord[] records

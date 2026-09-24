# dashboard

웹 기반 모니터링 대시보드(프론트엔드) 자리. 필요해지면 여기에 작성한다.

- ROS 패키지가 아니다. `COLCON_IGNORE`로 colcon 빌드에서 제외된다.
- ROS 데이터는 `rosbridge_server`(WebSocket) 또는 `foxglove_bridge`로 받는다. 대시보드 코드가 ROS 노드를 직접 import하지 않는다.
- 필요한 토픽은 [docs/api.md](../docs/api.md)의 인터페이스만 사용한다. 대시보드 전용 데이터가 필요하면 `amr_msgs`에 메시지를 추가하는 절차를 따른다.
- 시계열 KPI는 Prometheus/Grafana(`docker/grafana/`)가 담당하므로, 이 대시보드는 지도 위 로봇 위치·상태·작업·알림처럼 Grafana로 표현하기 어려운 화면에 집중한다.
- 실행은 docker-compose의 `dashboard` 프로필로 붙인다.

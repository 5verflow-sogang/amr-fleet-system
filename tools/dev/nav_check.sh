#!/bin/bash
# 호스트에서 한 줄로: dev 컨테이너 찾기(없으면 띄움) → 필요하면 빌드 → 스택 실행 → Nav2 활성 대기 → 목표 전송 →
# 결과 표 → 스택 종료. 로그는 tools/dev/logs/<시각>/ 에 복사한다 (launch.log, summary.txt).
#
#   tools/dev/nav_check.sh                                   # 2단계(Nav2+구역), 1대, 넓은 곳 주행 (20,-14)
#   tools/dev/nav_check.sh -r 2 -g amr_01:6,0                # 2대, amr_01을 map (6,0)으로
#   tools/dev/nav_check.sh -r 2 --stage 3 -g amr_01:6,0      # 추적·안전 노드까지
#   tools/dev/nav_check.sh -g amr_01:20,-14 -g amr_01:6,0    # 같은 로봇은 순서대로, 다른 로봇은 동시에
#   tools/dev/nav_check.sh --set dynamic_obstacles:=true     # launch 인자 추가
#   tools/dev/nav_check.sh --build ...                       # 코드·설정(yaml 포함)을 고쳤으면 먼저 빌드
#   tools/dev/nav_check.sh --keep ...                        # 끝나도 스택 유지 (RViz로 볼 때)
#   tools/dev/nav_check.sh --attach -g amr_01:6,0            # 이미 떠 있는 스택에 목표만
#   tools/dev/nav_check.sh --stop                            # --keep으로 남긴 스택 끄기
#   tools/dev/nav_check.sh --obstacles [amr_01]              # 떠 있는 스택에서 로봇 주변 추적 트랙 (DWA가 멈췄을 때)
#
# 목표 좌표는 map 기준 (x,y[,yaw rad]). 모든 목표가 SUCCEEDED면 exit 0.
# 컨테이너는 docker compose의 dev 서비스(./src → /ros2_ws/src)다. 이미 `docker compose run --rm dev`로 띄운 게 있으면 그걸 쓴다.
set -euo pipefail
# Windows Git Bash(MSYS)는 /ros2_ws/..., /tmp/... 같은 인자를 Windows 경로로 바꿔 docker exec·docker cp에 넘긴다
# (test -f /ros2_ws/install/setup.bash가 늘 실패해 매번 다시 빌드). 변환을 끈다. macOS·Linux에는 영향 없음
export MSYS_NO_PATHCONV=1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
HERE="$REPO/tools/dev"
BUILD=0
STOP=0
OBSTACLES=""
ARGS=()
for a in "$@"; do
  case "$a" in
    --obstacles) OBSTACLES=amr_01 ;;
    amr_[0-9][0-9]) if [ -n "$OBSTACLES" ]; then OBSTACLES="$a"; else ARGS+=("$a"); fi ;;
    --build) BUILD=1 ;;
    --stop) STOP=1 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) ARGS+=("$a") ;;
  esac
done

# 이 체크아웃의 compose 프로젝트 dev 컨테이너만 쓴다. 이름(dev-run)으로 찾으면 다른 워크트리 것을 잡는다 (#243)
PROJECT="$(cd "$REPO" && docker compose config 2>/dev/null | sed -n 's/^name: //p' | head -1)" || true
[ -n "$PROJECT" ] || { echo "docker compose 프로젝트 이름을 못 읽었다 ($REPO)"; exit 1; }
find_dev() {
  docker ps -q --filter "label=com.docker.compose.project=$PROJECT" \
    --filter label=com.docker.compose.service=dev | head -1
}
CID="$(find_dev)"
if [ -z "$CID" ]; then
  if [ "$STOP" = 1 ] || [ -n "$OBSTACLES" ]; then echo "dev 컨테이너가 없다"; exit 0; fi
  echo "dev 컨테이너가 없어 새로 띄운다 (빌드가 처음부터라 몇 분 걸린다)"
  (cd "$REPO" && docker compose --profile dev run -d --rm dev sleep infinity >/dev/null)
  CID="$(find_dev)"
  BUILD=1
fi
# shellcheck disable=SC2016  # 컨테이너 안에서 펼친다
SETUP='for s in /ros2_ws/install/setup.bash /opt/teb_ws/install/setup.bash /opt/ros/humble/setup.bash; do [ -f $s ] && { source $s; break; }; done'

if [ -n "$OBSTACLES" ]; then
  docker exec -i "$CID" bash -c "$SETUP; python3 - $OBSTACLES" < "$HERE/obstacles_near.py"
  exit
fi

if [ "$STOP" = 1 ]; then
  docker exec "$CID" bash -c "pkill -INT -f 'ros2 launch amr_bringup system.launch.py'; sleep 10; \
    pkill -9 -f 'ros2 launch amr_bringup system.launch.py'; pkill -9 -f 'ign gazebo'; pkill -9 -f 'ruby.*ign'" || true
  echo "스택을 껐다"
  exit 0
fi

if [ "$BUILD" = 1 ] || ! docker exec "$CID" test -f /ros2_ws/install/setup.bash; then
  echo "빌드: colcon build --packages-up-to amr_bringup (로그는 컨테이너 /tmp/nav_check_build.log)"
  docker exec "$CID" bash -c "source /opt/teb_ws/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; \
    cd /ros2_ws && colcon build --packages-up-to amr_bringup > /tmp/nav_check_build.log 2>&1 \
    || { tail -30 /tmp/nav_check_build.log; exit 1; }"
  echo "빌드 완료"
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="/tmp/nav_check/$STAMP"
COLOR_ARG=()
[ -t 1 ] || COLOR_ARG=(--no-color)

# 빈 배열은 ${a[@]+"${a[@]}"}로 넘긴다 (macOS bash 3.2는 set -u에서 빈 배열을 unbound로 본다)
# Ctrl+C: docker exec -i는 신호를 넘기지 않으므로 컨테이너 안 프로세스에 직접 보낸다 (스택 정리는 그쪽이 한다)
# shellcheck disable=SC2329  # trap으로 부른다
on_int() {
  docker exec "$CID" bash -c "kill -INT \$(cat $OUT/pid 2>/dev/null) 2>/dev/null" || true
}
trap on_int INT

# ipc: host라 /dev/shm을 모든 컨테이너가 같이 쓴다. 죽은 프로세스의 Fast DDS 파일이 쌓이면 /scan이 Nav2에 안 들어온다 (#243).
# 쓰는 중인 파일은 남기므로 --attach에서도 안전하다 (하네스 runner.py와 같다)
docker exec "$CID" bash -c "$SETUP; fastdds shm clean >/dev/null 2>&1" || true

set +e
docker exec -i "$CID" bash -c "$SETUP; python3 - \"\$@\"" _ --out "$OUT" ${COLOR_ARG[@]+"${COLOR_ARG[@]}"} ${ARGS[@]+"${ARGS[@]}"} \
  < "$HERE/nav_check.py"
CODE=$?
# Ctrl+C를 받은 뒤에도 안쪽이 정리를 끝낼 때까지 기다린다
while docker exec "$CID" bash -c "kill -0 \$(cat $OUT/pid 2>/dev/null) 2>/dev/null"; do sleep 1; done
set -e

mkdir -p "$HERE/logs"
if docker cp "$CID:$OUT" "$HERE/logs/$STAMP" >/dev/null 2>&1; then
  echo "로그 복사: tools/dev/logs/$STAMP/"
fi
exit "$CODE"

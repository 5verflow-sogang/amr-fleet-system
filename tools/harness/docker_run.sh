#!/bin/bash
# 호스트에서 하네스를 컨테이너로 돌린다. 레포를 /repo에 올리고, 필요한 패키지만 빌드한 뒤 run.py를 부른다.
# 결과는 호스트의 tools/harness/results/에 남는다.
#
#   tools/harness/docker_run.sh loc_basic -n 10
#   HARNESS_IMAGE=amr-fleet-system:latest tools/harness/docker_run.sh fleet_fake -n 5 --set delay_ms=200
#   HARNESS_FRESH=1 tools/harness/docker_run.sh loc_basic -n 10     # 반복마다 새 컨테이너
#
# HARNESS_IMAGE: 이미지 (기본 amr-fleet-system:latest)
# HARNESS_PREP:  빌드 전에 컨테이너에서 실행할 명령 (예: 이미지에 없는 apt 패키지 설치)
# HARNESS_PACKAGES: 빌드할 패키지 목록 (기본: loc_basic·fleet_fake에 필요한 것)
# HARNESS_DISCORD_WEBHOOK: 있으면 끝나고 요약을 Discord로 보낸다. 값은 명령줄에 안 남게 이름만 넘긴다
# HARNESS_RUNNER_MACHINE, HARNESS_RUNNER_SLOT: GCP 러너 VM 종류·슬롯 (fleet-test.yml이 준다). meta.json host와 Discord 요약에 남는다
# HARNESS_FRESH: 1이면 반복 하나마다 컨테이너를 새로 띄운다. 기본(한 컨테이너)도 시도마다 DDS 도메인을 바꾸고
#                흔적을 지우지만, 그래도 뒤 반복으로 갈수록 결과가 달라지는지 확인할 때 쓴다.
#                빌드 결과는 이번 실행 동안만 쓰는 볼륨에 두어 두 번째 컨테이너부터는 빌드가 거의 없다
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${HARNESS_IMAGE:-amr-fleet-system:latest}"
# HARNESS_PACKAGES: 빌드할 패키지(--packages-up-to). 없으면 시나리오 파일의 packages: 줄, 그것도 없으면 아래 기본값
# (loc_basic·fleet_fake용). 실제 로봇 스택 시나리오(fleet_stack 등)는 packages: amr_bringup을 적어 둔다
SCENARIO_PACKAGES=""
if [ $# -ge 1 ] && [ -f "$REPO/tools/harness/scenarios/$1.yaml" ]; then
  SCENARIO_PACKAGES="$(sed -n 's/^packages:[[:space:]]*//p' "$REPO/tools/harness/scenarios/$1.yaml" | head -1)"
fi
PACKAGES="${HARNESS_PACKAGES:-${SCENARIO_PACKAGES:-amr_gazebo amr_odometry amr_localization amr_fleet_manager amr_fault_injector}}"

# $1: 추가 docker 인자(공백 구분), 나머지: run.py 인자
run_container() {
  local extra="$1"; shift
  # --init: 부모가 먼저 끝난 프로세스(Gazebo 실행기 등)를 거둬 좀비가 쌓이지 않게 한다
  # shellcheck disable=SC2086
  docker run --rm --init -e HARNESS_IMAGE="$IMAGE" -e HARNESS_PREP="${HARNESS_PREP:-}" -e HARNESS_DISCORD_WEBHOOK \
    -e HARNESS_RUNNER_MACHINE -e HARNESS_RUNNER_SLOT $extra \
    -v "$REPO:/repo" -w /repo "$IMAGE" bash -c "
      set -e
      # 이미지의 가장 위 underlay를 source한다 (/opt/teb_ws → /opt/tracing_ws → /opt/ros, Dockerfile 체인).
      # /opt/ros만 source하면 TEB(teb_local_planner)와 LTTng tracetools가 안 보인다
      for s in /opt/teb_ws/install/setup.bash /opt/tracing_ws/install/setup.bash /opt/ros/\${ROS_DISTRO:-humble}/setup.bash; do
        if [ -f \$s ]; then source \$s; break; fi
      done
      # 레포 주인(호스트 사용자)과 컨테이너 사용자(root)가 달라 git이 거부하면 meta.json git_sha가 unknown이 된다
      git config --global --add safe.directory /repo 2>/dev/null || true
      if [ -n \"\$HARNESS_PREP\" ]; then bash -c \"\$HARNESS_PREP\"; fi
      colcon build --base-paths src --build-base /tmp/harness_ws/build --install-base /tmp/harness_ws/install \
        --packages-up-to $PACKAGES > /tmp/harness_build.log 2>&1 || { tail -30 /tmp/harness_build.log; exit 1; }
      source /tmp/harness_ws/install/setup.bash
      python3 tools/harness/run.py $(printf '%q ' "$@")"
}

if [ "${HARNESS_FRESH:-0}" != 1 ]; then
  run_container "" "$@"
  exit
fi

# 새 컨테이너 모드: -n을 빼서 반복마다 -n 1 --repeat-offset i로 부른다. 같은 --stamp라 마지막 요약이 전체를 묶는다
REPEATS=10
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -n|--repeats) REPEATS="$2"; shift 2 ;;
    -n*) REPEATS="${1#-n}"; shift ;;
    --repeats=*) REPEATS="${1#--repeats=}"; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
STAMP="$(date +%Y%m%d-%H%M%S)"
VOLUME="amr-harness-ws-$STAMP"
docker volume create "$VOLUME" > /dev/null
trap 'docker volume rm "$VOLUME" > /dev/null' EXIT
WEBHOOK="${HARNESS_DISCORD_WEBHOOK:-}"
for ((i = 0; i < REPEATS; i++)); do
  # Discord 요약은 마지막 반복에서만 보낸다
  if [ $((i + 1)) -lt "$REPEATS" ]; then export HARNESS_DISCORD_WEBHOOK=; else export HARNESS_DISCORD_WEBHOOK="$WEBHOOK"; fi
  run_container "-v $VOLUME:/tmp/harness_ws" "${ARGS[@]}" -n 1 --repeat-offset "$i" --stamp "$STAMP"
done

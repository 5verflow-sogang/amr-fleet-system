#!/bin/bash
# 하네스를 GCP 러너(fleet-test.yml)로 돌린다. docker_run.sh와 같은 인자를 받으니 스크립트 이름만 바꾸면 된다.
#
#   로컬: tools/harness/docker_run.sh nav_compare -n 10 --set controller=teb --label teb_obstacle
#   GCP:  tools/harness/gcp_run.sh    nav_compare -n 10 --set controller=teb --label teb_obstacle
#
# 돌릴 코드: 지금 브랜치의 열린 PR을 찾아 그 PR 코드로 돌린다 (push한 데까지). 없으면 --pr <번호> 또는 --ref dev
# HARNESS_FRESH=1, HARNESS_PACKAGES는 docker_run.sh처럼 환경 변수로 준다 (--fresh, --packages a,b도 된다)
# --slot a|b1~b4: 러너 슬롯 (기본 a = n2d-standard-16, b = c2d-standard-8). 여러 조건은 gcp_batch.py로
# --watch: 끝날 때까지 진행 상황을 보여 준다. 안 주면 실행 링크만 찍고 바로 끝난다
# 필요: gh (gh auth login)
set -euo pipefail
REPO_SLUG="${HARNESS_GH_REPO:-5verflow-sogang/amr-fleet-system}"
WORKFLOW=fleet-test.yml
# 워크플로 파일을 읽을 브랜치. 워크플로를 고친 PR을 머지 전에 시험할 때만 바꾼다 (원 저장소에 그 브랜치가 있어야 한다)
WORKFLOW_REF="${HARNESS_WORKFLOW_REF:-dev}"

usage() { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in -h|--help) usage 0 ;; -*) echo "첫 인자는 시나리오 이름 (예: loc_basic)" >&2; exit 1 ;; esac
scenario="$1"; shift

repeats=3; label=""; ref=""; watch=0; slot=a
fresh="${HARNESS_FRESH:-0}"; packages="${HARNESS_PACKAGES:-}"
# 지연·손실을 안 주면 시나리오 params 기본값을 쓴다 (워크플로 입력 기본값 0이 시나리오 기본값을 덮지 않게.
# 예: fleet_stack은 원 명세 조건인 지연 100 ms가 기본)
scenario_default() {
  local file
  file="$(dirname "$0")/scenarios/${scenario}.yaml"
  [ -f "$file" ] && sed -n "s/^  $1:[[:space:]]*\([0-9.]*\).*/\1/p" "$file" | head -1
}
delay="$(scenario_default delay_ms || true)"; delay="${delay:-0}"
loss="$(scenario_default loss_pct || true)"; loss="${loss:-0}"
kill_robot=none; kill_at=-1
params=()
while [ $# -gt 0 ]; do
  case "$1" in
    -n|--repeats) repeats="$2"; shift 2 ;;
    -n*) repeats="${1#-n}"; shift ;;
    --repeats=*) repeats="${1#--repeats=}"; shift ;;
    --label) label="$2"; shift 2 ;;
    # 장애 값은 워크플로의 전용 입력으로, 나머지는 params로 보낸다
    --set)
      case "$2" in
        delay_ms=*) delay="${2#*=}" ;;
        loss_pct=*) loss="${2#*=}" ;;
        kill_robot=*) kill_robot="${2#*=}" ;;
        kill_robot_at_s=*) kill_at="${2#*=}" ;;
        *=*) params+=("$2") ;;
        *) echo "--set은 이름=값: $2" >&2; exit 1 ;;
      esac
      shift 2 ;;
    --ref) ref="$2"; shift 2 ;;
    --pr) ref="refs/pull/$2/head"; shift 2 ;;
    --fresh) fresh=1; shift ;;
    --packages) packages="$2"; shift 2 ;;
    --slot) slot="$2"; shift 2 ;;
    --watch) watch=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "GCP에서는 못 쓰는 인자: $1 (-n, --set, --label, --pr, --ref, --fresh, --packages, --slot, --watch)" >&2; exit 1 ;;
  esac
done

case "$slot" in
  a) ;;
  b1|b2|b3|b4) params+=("slot=$slot") ;;
  *) echo "--slot은 a, b1~b4: $slot" >&2; exit 1 ;;
esac

if [ -n "$packages" ]; then
  # HARNESS_PACKAGES는 공백 구분, 워크플로 params는 쉼표 구분
  packages=$(echo "$packages" | tr -s ' ,' ',' | sed 's/^,//; s/,$//')
  params+=("packages=$packages")
fi

# 돌릴 코드. 팀원 브랜치는 포크에 있어 PR 번호로 가리킨다
if [ -z "$ref" ]; then
  branch=$(git -C "$(dirname "$0")" rev-parse --abbrev-ref HEAD)
  pr=$(gh pr list -R "$REPO_SLUG" --head "$branch" --state open --json number -q '.[0].number')
  if [ -z "$pr" ]; then
    echo "지금 브랜치($branch)의 열린 PR이 없다. --pr <번호> 또는 --ref dev로 정한다" >&2
    exit 1
  fi
  ref="refs/pull/$pr/head"
  # GCP는 GitHub에 올라간 코드를 받는다. 로컬에만 있는 커밋은 들어가지 않는다
  head=$(gh pr view "$pr" -R "$REPO_SLUG" --json headRefOid -q .headRefOid)
  if [ "$head" != "$(git -C "$(dirname "$0")" rev-parse HEAD)" ]; then
    echo "주의: 로컬 HEAD가 PR #${pr}과 다르다. push한 코드(${head:0:7})로 돈다" >&2
  fi
fi

fresh_flag=false
if [ "$fresh" = 1 ]; then fresh_flag=true; fi
params_str="${params[*]+"${params[*]}"}"

busy=$(gh run list -R "$REPO_SLUG" --workflow "$WORKFLOW" --json status -q '[.[] | select(.status != "completed")] | length')
echo "GCP 실행: $scenario × ${repeats}회, ref=$ref, 슬롯 $slot"
echo "  지연 ${delay} ms, 손실 ${loss} %, 로봇 장애 ${kill_robot}${label:+, 라벨 $label}${params_str:+, params: $params_str}, 새 컨테이너 $fresh_flag"
if [ "${busy:-0}" -gt 0 ]; then
  echo "  지금 ${busy}개가 돌거나 기다리는 중이다. 같은 슬롯이면 그 뒤에 시작하고, 같은 슬롯에 대기 중인 실행이 있으면 GitHub가 그것을 취소한다"
fi

# 방금 넣은 실행을 찾을 때 다른 사람이 같은 때 넣은 실행을 잡지 않게, 내 계정·워크플로 브랜치·수동 실행만 본다
# (10/16 슬롯 스모크 때 2초 뒤 다른 팀원 실행을 잘못 잡았다). 나 혼자 동시에 여러 개 넣는 건 gcp_batch.py가 순서대로 한다
me=$(gh api user -q .login)
mine=(--workflow "$WORKFLOW" --branch "$WORKFLOW_REF" --user "$me" --event workflow_dispatch)
before=$(gh run list -R "$REPO_SLUG" "${mine[@]}" -L 1 --json databaseId -q '.[0].databaseId // 0')
# 수동 실행은 dev의 워크플로 파일로 돈다(HARNESS_WORKFLOW_REF). 돌릴 코드는 입력 ref로 따로 준다
gh workflow run "$WORKFLOW" -R "$REPO_SLUG" --ref "$WORKFLOW_REF" \
  -f ref="$ref" -f scenario="$scenario" -f repeats="$repeats" \
  -f delay_ms="$delay" -f loss_pct="$loss" -f kill_robot="$kill_robot" -f kill_robot_at_s="$kill_at" \
  -f label="$label" -f params="$params_str" -f fresh="$fresh_flag"

# 방금 만든 실행을 찾는다 (등록까지 몇 초 걸린다)
run=""
for _ in 1 2 3 4 5 6 7 8 9 10; do
  sleep 2
  run=$(gh run list -R "$REPO_SLUG" "${mine[@]}" -L 1 --json databaseId -q '.[0].databaseId // 0')
  if [ "$run" != "$before" ]; then break; fi
  run=""
done
if [ -z "$run" ]; then
  echo "실행 링크를 못 찾았다: gh run list -R $REPO_SLUG --workflow $WORKFLOW"
  exit 0
fi
echo "실행: https://github.com/$REPO_SLUG/actions/runs/$run"
echo "  입력이 틀렸으면 1분 안에 실패하고 이유가 로그 맨 위에 나온다. 결과 요약은 Discord 실험 결과 채널로도 온다"
if [ "$watch" = 1 ]; then
  gh run watch -R "$REPO_SLUG" "$run" --exit-status
else
  echo "  진행 보기: gh run watch -R $REPO_SLUG $run"
fi

# GCP 일회용 러너 (tools/infra/gcp)

- 담당: 하연
- 책임: 연구 1·CI용 GCP 자원(Terraform)과 `fleet-test.yml`의 일회용 Spot 러너

명세서 11장의 역할 분담대로 연구 1·CI·5대 통합(카메라 끔)은 GCP `n2d-standard-16` Spot에서 돌린다. 10/5 실측에서 카메라를 끈 로봇 5대가 RTF 1.00으로 돌았다(명세서 11-3절).

## 구성

```
GitHub Actions (fleet-test.yml, 수동 실행)
  start-runner (ubuntu-latest)
    ├─ Workload Identity Federation으로 GCP 로그인 (키 파일 없음, fleet-gha 계정)
    ├─ GitHub API로 일회용(JIT) 러너 설정 발급 (RUNNER_ADMIN_TOKEN)
    └─ 템플릿 fleet-runner로 Spot VM 생성, 메타데이터 jit-config 전달
  fleet-test (그 VM)
    ├─ startup.sh: Docker·러너 설치 → 작업 하나만 받는 러너로 등록
    ├─ Docker 이미지 빌드 (Artifact Registry 빌드 캐시)
    ├─ 하네스 실행: tools/harness/docker_run.sh <시나리오> -n <반복> (기본 fleet_fake)
    ├─ 반복이 전부 실패하면 job 실패
    ├─ 결과·요약을 gs://amr-fleet-results/runs/<시나리오>-<라벨>/<날짜>_<실행 번호>/ 에 업로드 (fleet-runner-vm 계정),
    │  요약은 실행 화면 Summary에도
    └─ 실행 목록 gs://amr-fleet-results/index.csv 에 한 줄 (tools/harness/index_run.py)
  stop-runner (ubuntu-latest, 항상 실행)
    └─ VM 삭제, 남은 러너 등록 삭제
```

| 파일 | 내용 |
| --- | --- |
| `main.tf` | API, 결과 버킷 `amr-fleet-results`, Artifact Registry `amr` |
| `iam.tf` | 서비스 계정 `fleet-gha`(VM 생성·삭제), `fleet-runner-vm`(버킷·이미지 캐시), GitHub OIDC 연동, 팀원 결과 버킷 권한(`results_viewers`·`results_writers`) |
| `runner.tf` | 인스턴스 템플릿 `fleet-runner` (Spot, 최대 실행 시간 후 자동 삭제) |
| `budget.tf` | 월 예산 알림 50/90/100% (`billing_account_id`를 줄 때만) |
| `startup.sh` | VM 시작 스크립트 |

### 비용 안전장치

| 상황 | 장치 |
| --- | --- |
| 정상 종료·실패·취소 | `stop-runner`가 항상 VM을 지운다 |
| 워크플로 자체가 죽음 | 템플릿 `max_run_duration`(5시간)이 지나면 GCP가 VM을 지운다 |
| Spot 회수 | 종료 동작이 DELETE라 VM이 지워진다. 결과는 끝난 단계까지만 남는다 |
| 작업이 끝났는데 VM이 남음 | `startup.sh`가 러너 종료 뒤 VM을 끈다 (컴퓨팅 요금 멈춤) |
| 월 사용량 | 예산 알림 (무료 크레딧 차감 전 금액 기준) |

- N2D 한도가 16 vCPU라 러너는 동시에 1대다. 워크플로 `concurrency`로 줄 세운다.
- `n2d-standard-16` Spot 약 0.13달러/시간. 첫 실행은 이미지 빌드(tracetools 소스 빌드 포함)로 오래 걸리고, 이후에는 캐시를 쓴다.

## 처음 설정 (하연, 한 번)

### 1. Terraform 적용

```bash
gcloud auth application-default login     # Terraform이 쓰는 로그인
cd tools/infra/gcp
cp terraform.tfvars.example terraform.tfvars
# 예산 알림을 걸려면 billing_account_id를 채운다: gcloud billing accounts list
terraform init
terraform plan
terraform apply
terraform output                          # 2번에 넣을 값
```

- 상태 파일(`terraform.tfstate`)은 지금 로컬에만 있다. 잃어버리면 자원을 Terraform으로 관리할 수 없게 되므로 백업하거나, 나중에 GCS 백엔드로 옮긴다. 커밋하지 않는다(`.gitignore`).
- 예산 알림은 결제 계정에 대한 권한(결제 계정 관리자 또는 예산 관리자)이 있어야 만들어진다.

### 2. GitHub 저장소 설정 (`5verflow-sogang/amr-fleet-system`, 관리자 권한 필요)

| 종류 | 이름 | 값 |
| --- | --- | --- |
| 변수 (Settings → Secrets and variables → Actions → Variables) | `GCP_WIF_PROVIDER` | `terraform output GCP_WIF_PROVIDER` |
| 변수 | `GCP_GHA_SA` | `terraform output GCP_GHA_SA` |
| 시크릿 | `RUNNER_ADMIN_TOKEN` | 아래 토큰 |

`RUNNER_ADMIN_TOKEN`: 러너 등록에는 저장소 **Administration 쓰기** 권한이 필요하다. 기본 `GITHUB_TOKEN`으로는 안 된다.

- fine-grained 개인 토큰: Resource owner `5verflow-sogang`, 저장소 `amr-fleet-system`만, Repository permissions → Administration: Read and write. 조직이 fine-grained 토큰을 막아 두었으면 조직 설정에서 허용하거나 승인한다.
- 만료일을 MOST 마감 뒤(12월)로 둔다.

조직 설정 → Actions → Runners에서 저장소 단위 self-hosted 러너가 허용돼 있는지 확인한다.

### 3. 실행

**가장 쉬운 방법: 로컬 하네스 명령에서 `docker_run.sh`를 `gcp_run.sh`로 바꾼다.** 인자가 같다. `gh`(`gh auth login`)만 있으면 된다.

```bash
# 로컬
tools/harness/docker_run.sh loc_basic -n 10
# GCP (지금 브랜치의 PR 코드로 돈다. push한 데까지)
tools/harness/gcp_run.sh loc_basic -n 10

# 시나리오 값·라벨·새 컨테이너·빌드 패키지도 로컬과 같다
HARNESS_FRESH=1 HARNESS_PACKAGES="amr_bringup amr_nav2_plugins" \
  tools/harness/gcp_run.sh nav_compare -n 10 --set controller=teb --set scenario=obstacle --label teb_obstacle

# Fleet 장애도 --set으로
tools/harness/gcp_run.sh fleet_fake -n 10 --set delay_ms=200 --ref dev
```

| 옵션 | 뜻 |
| --- | --- |
| (없음) | 지금 브랜치의 열린 PR 코드로 돈다. PR이 없으면 멈추고 안내한다 |
| `--pr 145` / `--ref dev` | 다른 PR이나 브랜치 코드로 |
| `--watch` | 끝날 때까지 진행을 보여 준다. 없으면 실행 링크만 찍고 끝난다 |
| `--fresh`, `--packages a,b` | `HARNESS_FRESH=1`, `HARNESS_PACKAGES`와 같다 |

`gcp_run.sh`는 아래 워크플로 입력으로 바꿔 `gh workflow run`을 부를 뿐이다. 브라우저에서는 Actions → Fleet Regression Test → Run workflow에 같은 값을 넣는다. 저장소 관리자면 누구나 돌릴 수 있다.

| 입력 | 기본값 | 설명 |
| --- | --- | --- |
| `ref` | `dev` | 돌릴 코드. **포크 브랜치는 PR을 열고 `refs/pull/<번호>/head`** (머지 전 시나리오도 이렇게 돌린다) |
| `scenario` | `fleet_fake` | `tools/harness/scenarios/<이름>.yaml` |
| `repeats` | `3` | 1~15. 제한 시간 240분 안에 들게 정한다 |
| `delay_ms`, `loss_pct` | `0` | 통신 장애. 시나리오 `params`에 `delay_ms`가 있을 때만 넘기고, 없는 시나리오에 0이 아닌 값을 넣으면 시작하지 않는다 |
| `kill_robot`, `kill_robot_at_s` | `none`, `-1` | 로봇 장애. 시나리오에 `kill_robot`이 있을 때만 |
| `label` | (비움) | 요약·`index.csv`·결과 경로에서 조건을 구분하는 이름. 비우면 `d<지연>_l<손실>[_kill_<로봇>]`, 장애 없는 시나리오는 `default`. `params`로 조건을 바꾸면 직접 정해야 한다 |
| `params` | (비움) | 시나리오 값을 `이름=값`으로, 여러 개면 공백으로 나눈다. 시나리오 `params`에 없는 이름(오타)은 거부한다. 값에는 공백·따옴표·`$`를 못 쓴다 |
| `fresh` | 끔 | 반복마다 새 컨테이너 (`HARNESS_FRESH=1`). Nav2 시나리오를 여러 번 돌리면 켠다 (#138) |

`params`의 특별한 이름은 `packages=a,b` 하나다. 시나리오 값이 아니라 컨테이너에서 빌드할 패키지(`docker_run.sh`의 `HARNESS_PACKAGES`)로 넘긴다.

예시

| 하려는 것 | 입력 |
| --- | --- |
| 러너만 빨리 확인 (약 8분) | `repeats=1`, `params=duration_s=120`, `label=smoke` |
| Fleet 지연 200 ms × 10회 | `repeats=10`, `delay_ms=200` (라벨 `d200_l0`) |
| 위치추정 KPI (RTF 1.0) | `scenario=loc_basic`, `repeats=10` |
| DWA·TEB 비교 (PR #145) | `ref=refs/pull/145/head`, `scenario=nav_compare`, `params=controller=teb scenario=obstacle packages=amr_bringup,amr_nav2_plugins`, `label=teb_obstacle`, `fresh` 켬 |

진행 상황은 `gh run list -R 5verflow-sogang/amr-fleet-system --workflow fleet-test.yml`로 본다.

- 러너는 동시에 1대다. 앞 실행이 끝날 때까지 대기열에 선다. 10회면 시나리오에 따라 1~2시간 걸리므로 긴 실행은 팀 채널에 미리 알린다.
- 카메라가 필요한 측정(도킹 ArUco, YOLO FPS)은 GCP에 맞지 않는다. CPU 렌더링이라 카메라를 켜면 RTF가 0.37로 떨어진다(명세서 11-3절).
- 끝나면 요약이 실행 화면 Summary와 Discord 실험 결과 채널로 온다.
- 포크 PR의 `pull_request` 이벤트는 GCP 로그인 토큰을 받지 못해 PR 라벨로 자동 실행하지 않는다.
- 결과는 `gs://amr-fleet-results/runs/<시나리오>-<라벨>/<날짜>_<실행 번호>/`: `rosbag/`, `fleet_latency.csv`, `response_time.csv`, `assignments.csv`, `tasks.json`, `meta.json`(`fault.direction`, `observation_cutoff` 포함), 각 노드 로그.

`index.csv`(날짜, 시나리오, 라벨, 지연, 손실, 강제종료 로봇·시각, 경매 중계 여부, 반복 수, 완료 수, git SHA, 경로, Actions 링크)에서 원하는 실행을 찾고 경로로 내려받는다.

```bash
gcloud storage cat gs://amr-fleet-results/index.csv
gcloud storage ls gs://amr-fleet-results/runs/
gcloud storage cp -r gs://amr-fleet-results/runs/<시나리오>-<라벨>/<날짜>_<실행 번호> .
```

## 결과 보기 (팀원)

권한은 하연이 `terraform.tfvars`의 `results_viewers`(읽기)·`results_writers`(새 파일 올리기)에 Google 계정을 넣고 `terraform apply`로 준다.

- 브라우저: <https://console.cloud.google.com/storage/browser/amr-fleet-results> (권한 받은 Google 계정으로 로그인)
- 터미널 (`gcloud auth login` 후):

```bash
gcloud storage ls gs://amr-fleet-results/runs/
gcloud storage cp -r gs://amr-fleet-results/runs/<폴더> .
# 쓰기 권한이 있으면 로컬 하네스 결과 올리기 (같은 이름의 파일은 덮어쓸 수 없다)
gcloud storage cp -r tools/harness/results/<실행 폴더> gs://amr-fleet-results/runs/<폴더>/
```

## 문제 해결

| 증상 | 확인 |
| --- | --- |
| `fleet-test`가 러너를 계속 기다림 | VM 시리얼 로그: `gcloud compute instances get-serial-port-output <VM 이름> --zone asia-northeast3-a`. 시작 스크립트 로그는 VM에서 `sudo journalctl -u google-startup-scripts` |
| VM 생성 실패 `ZONE_RESOURCE_POOL_EXHAUSTED` | Spot 재고 부족. 다시 실행하거나 `ZONE`을 `asia-northeast3-b`·`c`로 바꾼다 |
| VM 생성 실패 `QUOTA_EXCEEDED` | 이전 VM이 남아 있다: `gcloud compute instances list` 후 지운다 |
| `generate-jitconfig` 403 | `RUNNER_ADMIN_TOKEN` 권한(Administration 쓰기)·만료 확인 |

## 정리

```bash
terraform destroy   # 버킷은 force_destroy=false라 비어 있어야 지워진다 (논문 원자료 보호)
```

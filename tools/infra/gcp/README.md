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
    ├─ 시나리오 실행 (지금은 smoke_fleet.sh, 하네스가 나오면 교체)
    └─ 결과를 gs://amr-fleet-results/runs/<VM 이름>/ 에 업로드 (fleet-runner-vm 계정)
  stop-runner (ubuntu-latest, 항상 실행)
    └─ VM 삭제, 남은 러너 등록 삭제
```

| 파일 | 내용 |
| --- | --- |
| `main.tf` | API, 결과 버킷 `amr-fleet-results`, Artifact Registry `amr` |
| `iam.tf` | 서비스 계정 `fleet-gha`(VM 생성·삭제), `fleet-runner-vm`(버킷·이미지 캐시), GitHub OIDC 연동 |
| `runner.tf` | 인스턴스 템플릿 `fleet-runner` (Spot, 최대 실행 시간 후 자동 삭제) |
| `budget.tf` | 월 예산 알림 50/90/100% (`billing_account_id`를 줄 때만) |
| `startup.sh` | VM 시작 스크립트 |
| `smoke_fleet.sh` | 스모크 시나리오: Fleet 서버 + 장애 주입기 + 가짜 로봇 5대 + 작업 생성기, `/fleet` 토픽 rosbag |

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

Actions → Fleet Regression Test → Run workflow. `ref`에 브랜치나 `refs/pull/<번호>/head`를 넣는다.

- 포크 PR의 `pull_request` 이벤트는 GCP 로그인 토큰을 받지 못해 PR 라벨로 자동 실행하지 않는다.
- 결과는 `gs://amr-fleet-results/runs/<VM 이름>/`: `rosbag/`, `assignments.csv`, `tasks.json`, `meta.json`, 각 노드 로그.

```bash
gcloud storage ls gs://amr-fleet-results/runs/
gcloud storage cp -r gs://amr-fleet-results/runs/<VM 이름> .
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

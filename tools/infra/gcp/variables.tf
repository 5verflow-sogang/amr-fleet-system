variable "project_id" {
  description = "GCP 프로젝트 ID"
  type        = string
  default     = "amr-fleet"
}

variable "region" {
  description = "자원 리전. 서울"
  type        = string
  default     = "asia-northeast3"
}

variable "zone" {
  description = "러너 VM 존"
  type        = string
  default     = "asia-northeast3-a"
}

variable "github_repository" {
  description = "러너를 쓰는 GitHub 저장소 (owner/name). 이 저장소의 워크플로만 GCP에 로그인할 수 있다"
  type        = string
  default     = "5verflow-sogang/amr-fleet-system"
}

variable "machine_type" {
  description = "러너 VM 머신. 카메라를 끈 로봇 5대가 RTF 1.0으로 도는 것을 10/5에 확인 (명세서 11-3절)"
  type        = string
  default     = "n2d-standard-16"
}

variable "disk_size_gb" {
  description = "러너 VM 부팅 디스크 크기 (Docker 이미지·rosbag)"
  type        = number
  default     = 100
}

variable "max_run_hours" {
  description = "러너 VM 최대 실행 시간. 지나면 GCP가 VM을 지운다 (워크플로가 지우지 못했을 때의 안전장치)"
  type        = number
  default     = 5
}

variable "results_bucket" {
  description = "실험 결과 버킷 이름 (전 세계에서 유일해야 함)"
  type        = string
  default     = "amr-fleet-results"
}

variable "billing_account_id" {
  description = "예산 알림을 걸 결제 계정 ID (XXXXXX-XXXXXX-XXXXXX). 비우면 예산 알림을 만들지 않는다"
  type        = string
  default     = ""
}

variable "budget_amount" {
  description = "월 예산 (budget_currency 단위). 50/90/100%에서 알림"
  type        = number
  default     = 50000
}

variable "budget_currency" {
  description = "예산 통화. 결제 계정 통화와 같아야 한다"
  type        = string
  default     = "KRW"
}

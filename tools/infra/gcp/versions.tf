terraform {
  required_version = ">= 1.6"

  # 상태 파일은 버킷에 둔다 (state.tf). 처음 한 번 terraform init -migrate-state로 로컬 상태를 옮긴다 (README "상태 파일")
  backend "gcs" {
    bucket = "amr-fleet-tfstate"
    prefix = "gcp"
  }

  required_providers {
    google = {
      source  = "hashicorp/google"
      version = "~> 6.0"
    }
  }
}

provider "google" {
  project = var.project_id
  region  = var.region
  zone    = var.zone

  # 예산(billingbudgets) API는 요청을 보내는 쪽 프로젝트로 할당량을 센다
  user_project_override = true
  billing_project       = var.project_id
}

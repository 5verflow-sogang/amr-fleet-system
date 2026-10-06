terraform {
  required_version = ">= 1.6"

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

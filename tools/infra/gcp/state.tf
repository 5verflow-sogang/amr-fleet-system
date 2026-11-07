# Terraform 상태 파일 버킷. 상태가 한 사람의 노트북에만 있으면 잃어버렸을 때 인프라를 관리할 수 없고
# 다른 사람이 plan·apply할 수 없다. 버전을 남겨 잘못 덮어써도 되돌린다 (backend 설정은 versions.tf)
resource "google_storage_bucket" "tfstate" {
  name                        = "${var.project_id}-tfstate"
  location                    = var.region
  uniform_bucket_level_access = true
  public_access_prevention    = "enforced"
  force_destroy               = false

  versioning {
    enabled = true
  }

  # 오래된 상태 버전은 30개까지만 남긴다
  lifecycle_rule {
    condition {
      num_newer_versions = 30
      with_state         = "ARCHIVED"
    }
    action {
      type = "Delete"
    }
  }

  depends_on = [google_project_service.this]
}

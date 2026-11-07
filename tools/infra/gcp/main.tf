# 계속 두는 자원: API, 결과 버킷, Docker 이미지 캐시. 러너 VM은 fleet-test.yml이 만들고 지운다.

# 프로젝트 번호(예산 알림 필터용). Cloud Resource Manager API가 켜진 뒤에 읽어야 하므로 API 켜기 뒤로 미룬다.
# 미루지 않으면 새 프로젝트에서 첫 plan이 403으로 실패한다
data "google_project" "this" {
  depends_on = [google_project_service.this]
}

locals {
  services = concat([
    "compute.googleapis.com",
    "iam.googleapis.com",
    "iamcredentials.googleapis.com",
    "sts.googleapis.com",
    "storage.googleapis.com",
    "artifactregistry.googleapis.com",
    "cloudresourcemanager.googleapis.com",
    "bigquery.googleapis.com",
  ], var.billing_account_id == "" ? [] : ["billingbudgets.googleapis.com"])
}

resource "google_project_service" "this" {
  for_each = toset(local.services)
  service  = each.value
  # destroy해도 API는 끄지 않는다 (같은 프로젝트의 다른 자원이 쓸 수 있음)
  disable_on_destroy = false
}

# 실행마다 결과(rosbag, CSV, meta.json)를 올린다. Spot이 회수돼도 끝난 반복의 결과는 남는다 (명세서 13장)
resource "google_storage_bucket" "results" {
  name                        = var.results_bucket
  location                    = var.region
  uniform_bucket_level_access = true
  public_access_prevention    = "enforced"
  # 논문 원자료라 실수로 지우지 않게 한다
  force_destroy = false

  # 시연 녹화 카메라 원본(demo_* camera_bag/, 장면당 약 8 GB)은 mp4 변환에 실패했을 때만 올라온다.
  # 변환기를 고쳐 다시 바꿀 시간(3일)만 두고 지운다. 측정용 rosbag/(rosbag_0.db3)은 이름이 달라 남는다
  lifecycle_rule {
    condition {
      age            = 3
      matches_suffix = ["camera_bag_0.db3"]
    }
    action {
      type = "Delete"
    }
  }

  depends_on = [google_project_service.this]
}

# 러너 VM은 매번 새로 뜨므로 Docker 빌드 캐시를 여기에 둔다
resource "google_artifact_registry_repository" "docker" {
  repository_id = "amr"
  location      = var.region
  format        = "DOCKER"
  description   = "러너용 Docker 이미지·빌드 캐시"

  # KEEP 규칙은 DELETE 규칙의 예외일 뿐이라, DELETE 규칙이 없으면 아무것도 지우지 않는다.
  # 처음엔 KEEP만 있어서 실행마다 올린 빌드 캐시가 쌓였다 (11/2 66개, 약 40 GB, 월 약 5,000원)
  cleanup_policies {
    id     = "delete-old"
    action = "DELETE"
    condition {
      older_than = "604800s" # 7일
    }
  }

  cleanup_policies {
    id     = "keep-recent"
    action = "KEEP"
    most_recent_versions {
      keep_count = 5
    }
  }

  depends_on = [google_project_service.this]
}

# 서비스 계정 두 개
# - fleet-gha: GitHub Actions가 키 없이(Workload Identity Federation) 쓰는 계정. 러너 VM 만들기·지우기만 한다
# - fleet-runner-vm: 러너 VM에 붙는 계정. 결과 버킷 쓰기, 이미지 캐시 읽기·쓰기

resource "google_service_account" "gha" {
  account_id   = "fleet-gha"
  display_name = "GitHub Actions: 러너 VM 생성·삭제"
  depends_on   = [google_project_service.this]
}

resource "google_service_account" "runner_vm" {
  account_id   = "fleet-runner-vm"
  display_name = "러너 VM: 결과 업로드·이미지 캐시"
  depends_on   = [google_project_service.this]
}

resource "google_project_iam_member" "gha_compute" {
  project = var.project_id
  role    = "roles/compute.instanceAdmin.v1"
  member  = "serviceAccount:${google_service_account.gha.email}"
}

# VM에 fleet-runner-vm 계정을 붙이려면 그 계정의 사용자 권한이 있어야 한다
resource "google_service_account_iam_member" "gha_uses_runner_vm" {
  service_account_id = google_service_account.runner_vm.name
  role               = "roles/iam.serviceAccountUser"
  member             = "serviceAccount:${google_service_account.gha.email}"
}

resource "google_storage_bucket_iam_member" "runner_vm_results" {
  bucket = google_storage_bucket.results.name
  role   = "roles/storage.objectAdmin"
  member = "serviceAccount:${google_service_account.runner_vm.email}"
}

resource "google_artifact_registry_repository_iam_member" "runner_vm_docker" {
  location   = google_artifact_registry_repository.docker.location
  repository = google_artifact_registry_repository.docker.name
  role       = "roles/artifactregistry.writer"
  member     = "serviceAccount:${google_service_account.runner_vm.email}"
}

resource "google_project_iam_member" "runner_vm_logging" {
  project = var.project_id
  role    = "roles/logging.logWriter"
  member  = "serviceAccount:${google_service_account.runner_vm.email}"
}

# GitHub OIDC → GCP. 키 파일을 시크릿에 두지 않는다
resource "google_iam_workload_identity_pool" "github" {
  workload_identity_pool_id = "github"
  display_name              = "GitHub Actions"
  depends_on                = [google_project_service.this]
}

resource "google_iam_workload_identity_pool_provider" "github" {
  workload_identity_pool_id          = google_iam_workload_identity_pool.github.workload_identity_pool_id
  workload_identity_pool_provider_id = "github-oidc"
  display_name                       = "GitHub OIDC"

  attribute_mapping = {
    "google.subject"       = "assertion.sub"
    "attribute.repository" = "assertion.repository"
    "attribute.ref"        = "assertion.ref"
  }
  # 이 저장소의 워크플로만 들어올 수 있다
  attribute_condition = "assertion.repository == '${var.github_repository}'"

  oidc {
    issuer_uri = "https://token.actions.githubusercontent.com"
  }
}

resource "google_service_account_iam_member" "gha_wif" {
  service_account_id = google_service_account.gha.name
  role               = "roles/iam.workloadIdentityUser"
  member             = "principalSet://iam.googleapis.com/${google_iam_workload_identity_pool.github.name}/attribute.repository/${var.github_repository}"
}

# 팀원 결과 열람. objectViewer는 객체만 읽어서, 콘솔에서 버킷을 열려면 버킷 정보 읽기(legacyBucketReader)도 준다
locals {
  results_readers = toset(concat(var.results_viewers, var.results_writers))
  results_reader_bindings = {
    for pair in setproduct(local.results_readers, ["roles/storage.objectViewer", "roles/storage.legacyBucketReader"]) :
    "${pair[0]} ${pair[1]}" => { email = pair[0], role = pair[1] }
  }
}

resource "google_storage_bucket_iam_member" "team_results_read" {
  for_each = local.results_reader_bindings
  bucket   = google_storage_bucket.results.name
  role     = each.value.role
  member   = "user:${each.value.email}"
}

# 팀원 결과 올리기 (로컬 하네스 결과). 새 파일만 만들 수 있고 덮어쓰기·삭제는 못 한다
resource "google_storage_bucket_iam_member" "team_results_write" {
  for_each = toset(var.results_writers)
  bucket   = google_storage_bucket.results.name
  role     = "roles/storage.objectCreator"
  member   = "user:${each.value}"
}

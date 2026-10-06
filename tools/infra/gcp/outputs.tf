# apply 뒤 GitHub 저장소 변수(Settings → Secrets and variables → Actions → Variables)에 넣는다

output "GCP_WIF_PROVIDER" {
  description = "저장소 변수 GCP_WIF_PROVIDER"
  value       = google_iam_workload_identity_pool_provider.github.name
}

output "GCP_GHA_SA" {
  description = "저장소 변수 GCP_GHA_SA"
  value       = google_service_account.gha.email
}

output "results_bucket" {
  value = google_storage_bucket.results.url
}

output "docker_repository" {
  value = "${var.region}-docker.pkg.dev/${var.project_id}/${google_artifact_registry_repository.docker.repository_id}"
}

output "instance_template" {
  value = google_compute_instance_template.runner.name
}

# 러너 VM 틀. fleet-test.yml이 이 템플릿으로 VM을 만들고, 작업이 끝나면 지운다.

resource "google_compute_instance_template" "runner" {
  name         = "fleet-runner"
  description  = "fleet-test.yml 일회용 러너 (Spot)"
  machine_type = var.machine_type

  disk {
    source_image = "ubuntu-os-cloud/ubuntu-2204-lts"
    disk_size_gb = var.disk_size_gb
    disk_type    = "pd-balanced"
    boot         = true
    auto_delete  = true
  }

  network_interface {
    network = "default"
    # GitHub·apt·Docker Hub에 나가야 해서 외부 IP를 준다. 들어오는 포트는 열지 않는다
    access_config {}
  }

  service_account {
    email  = google_service_account.runner_vm.email
    scopes = ["cloud-platform"]
  }

  scheduling {
    provisioning_model          = "SPOT"
    preemptible                 = true
    automatic_restart           = false
    on_host_maintenance         = "TERMINATE"
    instance_termination_action = "DELETE"
    # 워크플로가 VM을 지우지 못해도 이 시간이 지나면 GCP가 지운다
    max_run_duration {
      seconds = var.max_run_hours * 3600
    }
  }

  metadata = {
    # GitHub 러너를 설치하고 메타데이터 jit-config로 등록한다. 작업 하나를 끝내면 VM을 끈다
    startup-script = file("${path.module}/startup.sh")
  }

  labels = {
    purpose = "fleet-runner"
  }

  depends_on = [google_project_service.this]
}

# 실험 모니터링: 실행 목록(결과 버킷 index.csv)과 결제 데이터를 BigQuery에서 같이 본다.
# Looker Studio 대시보드는 fleet_ops의 뷰를 읽는다 (README "모니터링").
#
# index.csv는 외부 테이블이라 버킷 파일을 그대로 읽는다. 실험이 끝나 한 줄이 더해지면 바로 보인다(적재 작업 없음).
# 새 열은 끝에만 더해지므로(tools/harness/harness/index.py INDEX_HEADER) 모르는 열은 무시하고, 짧은 줄은 빈 값으로 둔다.

locals {
  index_columns = ["date", "scenario", "label", "delay_ms", "loss_pct", "kill_robot", "kill_robot_at_s",
  "relay_auction", "repeats", "completed", "git_sha", "path", "actions_url", "machine_type", "triggered_by"]
}

resource "google_bigquery_dataset" "ops" {
  dataset_id  = "fleet_ops"
  location    = var.region # 결과 버킷과 같은 위치여야 외부 테이블로 읽는다
  description = "실험 모니터링: 실행 목록(index.csv)·실행별 비용 뷰"

  depends_on = [google_project_service.this]
}

resource "google_bigquery_table" "runs_index" {
  dataset_id          = google_bigquery_dataset.ops.dataset_id
  table_id            = "runs_index"
  description         = "gs://${var.results_bucket}/index.csv 그대로 (모든 열 문자열). 형 변환은 runs 뷰에서"
  deletion_protection = false # 외부 테이블이라 지워도 원본(버킷)은 남는다

  external_data_configuration {
    source_format         = "CSV"
    autodetect            = false
    source_uris           = ["gs://${var.results_bucket}/index.csv"]
    ignore_unknown_values = true
    schema                = jsonencode([for c in local.index_columns : { name = c, type = "STRING", mode = "NULLABLE" }])

    csv_options {
      quote             = "\""
      skip_leading_rows = 1
      allow_jagged_rows = true
    }
  }
}

resource "google_bigquery_table" "runs" {
  dataset_id          = google_bigquery_dataset.ops.dataset_id
  table_id            = "runs"
  description         = "실행 한 번(N회 반복) = 한 줄. run_id는 GitHub Actions 실행 번호 = 러너 VM 라벨 run (#314)"
  deletion_protection = false

  view {
    use_legacy_sql = false
    query          = <<-SQL
      SELECT
        SAFE.PARSE_TIMESTAMP('%Y-%m-%dT%H:%M:%SZ', date) AS finished_at,
        scenario,
        label,
        SAFE_CAST(NULLIF(delay_ms, '') AS FLOAT64) AS delay_ms,
        SAFE_CAST(NULLIF(loss_pct, '') AS FLOAT64) AS loss_pct,
        NULLIF(kill_robot, '') AS kill_robot,
        SAFE_CAST(repeats AS INT64) AS repeats,
        SAFE_CAST(completed AS INT64) AS completed,
        SAFE_DIVIDE(SAFE_CAST(completed AS INT64), SAFE_CAST(repeats AS INT64)) AS completed_rate,
        NULLIF(git_sha, 'unknown') AS git_sha,
        path,
        actions_url,
        REGEXP_EXTRACT(actions_url, r'/runs/([0-9]+)') AS run_id,
        COALESCE(NULLIF(machine_type, ''), '(기록 전)') AS machine_type,
        COALESCE(NULLIF(triggered_by, ''), '(기록 전)') AS triggered_by
      FROM `${var.project_id}.${google_bigquery_dataset.ops.dataset_id}.${google_bigquery_table.runs_index.table_id}`
    SQL
  }
}

resource "google_bigquery_table" "runs_daily" {
  dataset_id          = google_bigquery_dataset.ops.dataset_id
  table_id            = "runs_daily"
  description         = "날짜(KST)·시나리오별 실행 수, 반복 수, 완료율"
  deletion_protection = false

  view {
    use_legacy_sql = false
    query          = <<-SQL
      SELECT
        DATE(finished_at, 'Asia/Seoul') AS day,
        scenario,
        COUNT(*) AS runs,
        SUM(repeats) AS repeats,
        SUM(completed) AS completed,
        SAFE_DIVIDE(SUM(completed), SUM(repeats)) AS completed_rate
      FROM `${var.project_id}.${google_bigquery_dataset.ops.dataset_id}.${google_bigquery_table.runs.table_id}`
      GROUP BY day, scenario
    SQL
  }
}

# 결제 내보내기(콘솔: 결제 → 결제 내보내기 → BigQuery 내보내기 → 표준 사용량 비용)가 여기에 테이블을 만든다.
# fleet_ops와 같은 위치여야 실행 목록과 조인된다. 실행별 비용 뷰는 내보내기 테이블이 생긴 뒤에 더한다
resource "google_bigquery_dataset" "billing" {
  count       = var.billing_account_id == "" ? 0 : 1
  dataset_id  = "billing_export"
  location    = var.region
  description = "Cloud Billing 표준 사용량 비용 내보내기 (콘솔에서 켠다)"

  depends_on = [google_project_service.this]
}

# 월 예산 알림 (명세서 11-2절: GCP·AWS 모두 예산 알림). billing_account_id를 줄 때만 만든다.
# 무료 체험 크레딧을 빼기 전 금액으로 잰다. 크레딧이 얼마나 빨리 줄어드는지 보려는 것이다.

resource "google_billing_budget" "monthly" {
  count = var.billing_account_id == "" ? 0 : 1

  billing_account = var.billing_account_id
  display_name    = "${var.project_id} 월 예산"

  budget_filter {
    projects               = ["projects/${data.google_project.this.number}"]
    credit_types_treatment = "EXCLUDE_ALL_CREDITS"
  }

  amount {
    specified_amount {
      currency_code = var.budget_currency
      units         = tostring(var.budget_amount)
    }
  }

  threshold_rules {
    threshold_percent = 0.5
  }
  threshold_rules {
    threshold_percent = 0.9
  }
  threshold_rules {
    threshold_percent = 1.0
  }

  depends_on = [google_project_service.this]
}

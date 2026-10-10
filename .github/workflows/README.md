# GitHub Actions

| 워크플로 | 언제 도나 | 하는 일 | 제한 시간 |
|---|---|---|---|
| `ci.yml` | `dev`·`main` push, PR (문서만 바뀌면 건너뜀) | colcon 빌드·경고 검사·테스트, 하네스 단위 테스트 | 30분 |
| `tracing-check.yml` | Dockerfile·트레이싱 스크립트가 바뀐 PR, 수동 | LTTng 이벤트 수집 확인 | 20분 |
| `rl-env.yml` | `experiments/research2/`가 바뀐 PR, 수동 | uv 설치, 환경 테스트, PPO 스모크 | 20분 |
| `fleet-test.yml` | 수동만 | GCP Spot VM에 일회용 러너를 띄워 Fleet 시험 후 결과를 GCS에 올림 | 러너 10분, 시험 240분 |
| `notify-discord.yml` | 위 워크플로가 끝날 때 | `dev`·`main` push, 수동·예약 실행이 실패하면 Discord에 알림 (PR 실패는 안 보냄) | 5분 |

액션 버전은 Dependabot(`.github/dependabot.yml`)이 매달 한 PR로 올린다.

## 실패했을 때

- **CI 빌드 경고**: `Check build warnings` 단계가 경고 난 패키지의 `stderr.log`를 출력한다. 경고도 실패로 본다 (원 명세서).
- **CI 테스트**: 실패하면 `colcon-logs` 아티팩트가 올라간다. launch 테스트가 가끔만 실패하면 병렬 테스트끼리 토픽이 섞였는지 먼저 본다 (`server_id` 등으로 걸러야 한다).
- **PR을 다시 돌렸는데 그대로 실패**: 다시 실행은 처음 만든 merge 커밋을 쓴다. `dev`에 고친 게 들어갔다면 PR 브랜치를 갱신(Update branch)한다.
- **`fleet-test`가 대기열에 멈춤**: 러너가 붙지 않은 것이다. VM 직렬 포트 로그를 본다
  (`gcloud compute instances get-serial-port-output <이름> --zone asia-northeast3-a --project amr-fleet`).
  실행을 취소해도 `stop-runner`가 VM과 러너 등록을 지운다.

## Discord 알림

- 웹훅 주소는 저장소 secret `DISCORD_WEBHOOK_URL`에 둔다 (Discord 채널 설정 → 연동 → 웹후크 → 새 웹후크 → URL 복사, `gh secret set DISCORD_WEBHOOK_URL`).
- 새 워크플로를 만들면 `notify-discord.yml`의 `workflows:` 목록에 그 `name:`을 더한다.
- 알림 워크플로는 `dev`에 있는 파일로 돈다. 고친 내용은 머지 뒤부터 적용된다.

## 규칙

- 브랜치 보호(필수 체크)는 비공개 무료 저장소라 쓸 수 없다. **CI가 초록일 때만 머지한다.**
- 새 job에는 `timeout-minutes`를 단다. 없으면 멈춘 작업이 6시간 동안 실행 시간을 쓴다.

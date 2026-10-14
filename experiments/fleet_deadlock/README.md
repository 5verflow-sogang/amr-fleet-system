# 교착 유도 실험 (Fleet KPI "교착 없음")

원 명세 9-1 "5대 운용 시 교착 없음"의 근거를 만든다. 일반 운용(`fleet_stack`)에서 교착이 안 생긴 것만으로는 해소 기능을 보였다고 할 수 없다. 그래서 일부러 교착을 만들고 서버 해소 전략이 풀어내는지 본다.

## 방법

- **구역:** 좁은 통로 ②(world x −4.4~0.6, 폭 0.6 m)를 가운데(x −1.9)에서 `narrow_02w`·`narrow_02e` 두 구역으로 나눈다(`zones_split.yaml`). 서버와 로봇(구역 클라이언트)이 같은 파일을 쓴다(`system.launch.py zones_file:=`).
- **로봇:** 2대를 통로 양 끝 바깥에 스폰한다(`robots.yaml`). `drive.py`가 두 로봇에 `NavigateThroughPoses`를 동시에 보낸다. 경유점이 두 구역을 차례로 지나게 해서, 전역 계획이 통로를 피해 넓은 길로 돌아가지 못하게 한다.
- **교착이 생기는 과정:** 각자 가까운 절반을 먼저 허가받고(`GRANTED`), 반대쪽 절반은 상대가 쥐고 있어 대기한다(`QUEUED`). 이 순환 대기가 교착이다.
- **해소 전략** (`deadlock_strategy`)
  - `revoke`(기본): 우선순위가 낮은 로봇의 허가를 회수한다(`REVOKED`). 그 로봇은 구역 클라이언트가 직접 후진해 빠져나간 뒤 반납한다(#137).
  - `avoid`: 순환을 닫게 될 요청을 거부한다(`REJECTED`). 로봇 쪽 우회 재계획은 아직 없다. 10/19 회의 안건이다.

## 실행

```bash
tools/harness/gcp_run.sh fleet_deadlock -n 3 --label revoke
tools/harness/gcp_run.sh fleet_deadlock -n 3 --set deadlock_strategy=avoid --label avoid
```

## 결과 파일

실행 폴더의 `deadlock_result.json`이다.

| 키 | 뜻 |
| --- | --- |
| `success` | 두 로봇 모두 `SUCCEEDED` (교착이 생겼다면 해소됨) |
| `deadlock_seen` | `REVOKED` 또는 `REJECTED`를 받은 로봇이 있음 (교착이 실제로 생김) |
| `time_s` | 늦게 끝난 로봇 기준 걸린 시간 (시뮬레이션 시간) |
| `robots.<id>.grants` | 구역 허가 답이 바뀐 순간들 (시각, 구역, 답, 양보받는 로봇) |

같은 실행의 `zone_waits.csv`(`fleet_metrics`)에는 구역별 대기 시간이 있다.

## 알려진 위험 (첫 실행에서 확인)

- **좁은 통로 진입:** #165(DWA lookahead 0.5·lethal 254)가 머지되기 전에는 통로 앞에서 DWA가 멈출 수 있다.
- **후진한 로봇이 통로 입구를 막음:** `revoke`로 후진한 로봇은 구역 경계 바로 밖(`backing_margin_m` 0.3 m)에서 다시 기다린다. 통로를 빠져나오는 상대 로봇의 진행 방향 정면이라, 몸체 길이(0.6 m)만큼 비켜 주지 않으면 물리적으로 막힐 수 있다.

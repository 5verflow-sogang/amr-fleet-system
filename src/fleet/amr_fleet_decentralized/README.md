# amr_fleet_decentralized

- 담당: 하연, 이진
- 빌드 타입: ament_python
- 책임: 분산 협상 기반 작업 할당과 중앙 서버 장애 시 폴백 (연구 1 비교군)

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | | | |
| Subscribe | | | |
| Service | | | |
| Action | | | |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

## 실행

```bash
# TODO
```

## 테스트

```bash
colcon test --packages-select amr_fleet_decentralized
```

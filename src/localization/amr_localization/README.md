# amr_localization

- 담당: 민지
- 빌드 타입: ament_python
- 책임: AMCL/EKF 설정, Kidnapped Robot 복구, GT 대비 RMSE 측정. 지도는 `amr_mapping`에서 받는다

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
colcon test --packages-select amr_localization
```

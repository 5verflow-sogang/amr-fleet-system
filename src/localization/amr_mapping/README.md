# amr_mapping

- 담당: 지수
- 빌드 타입: ament_python
- 책임: slam_toolbox 설정, 지도 저장(map.pgm/map.yaml), 맵 품질 평가. 저장한 지도는 `amr_localization`(민지)이 사용하며, 인계 조건은 [명세서 8-2절](../../../docs/project-spec.md#8-2-매핑위치추정-분업-지수--민지)을 따른다

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
colcon test --packages-select amr_mapping
```

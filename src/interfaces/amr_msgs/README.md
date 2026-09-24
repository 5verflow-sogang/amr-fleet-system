# amr_msgs

- 담당: 하연 (전원 리뷰)
- 빌드 타입: ament_cmake (rosidl)
- 책임: 전체 패키지가 공유하는 커스텀 msg/srv/action 정의

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
colcon test --packages-select amr_msgs
```

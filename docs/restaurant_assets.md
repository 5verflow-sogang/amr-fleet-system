# 식당 시연 3D 에셋 목록 (7주차, 민지 → 지수)

> 식당(뷔페) 월드의 겉모양용 메시 목록이다. **충돌체는 메시를 쓰지 않고 단순 박스로 둔다**(D2: 테이블은 바닥까지 꽉 찬 박스). 메시는 `<visual>`에만 넣는다.

## 1. 목록

| 용도 | 모델 | 출처 | 작성자 | 라이선스 | 받는 형식 → 넣을 형식 | 면 수 | 비고 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 배식대 받침 | Buffet | [Sketchfab](https://sketchfab.com/3d-models/55a5f4ddffc04e609e44bd8bb4bdd450) | nicolas saude | CC-BY 4.0 | glTF → OBJ | 6,578 | 스테인리스 배식 라인 (보온대·진열장 포함) |
| 배식대 위 | Buffets (food warmer), Foods, and Plates | [Sketchfab](https://sketchfab.com/3d-models/aa3a0d1b9baf4006a7ecf9e9f905ee9d) | sakigakefuruzawa | CC-BY 4.0 | glTF → OBJ | 12,020 | 음식 담긴 보온 용기 줄 + 접시 더미. 받침 위에 올린다 |
| 배식대 (대안) | cafeteria food warmer | [Sketchfab](https://sketchfab.com/3d-models/c321d257a044422096318def8d1352b2) | House Doctor | CC-BY 4.0 | glTF → OBJ | 7,336 | 유리 가드 보온대. 위 둘이 안 맞을 때 |
| 반납대 받침 | IKEA KUNGSFORS kitchen trolley | [Sketchfab](https://sketchfab.com/3d-models/5272c4bafd3044bcad2cea031a57a141) | WINTER | CC-BY 4.0 | glTF → OBJ | 18,979 | 스테인리스 3단 카트 |
| 반납대 위 | Dish Rack with Dishes | [Sketchfab](https://sketchfab.com/3d-models/ce69014964624fbb808a028e5c55d88b) | HippoStance | CC-BY 4.0 | glTF → OBJ | 8,640 | 식기 꽂힌 건조대. 카트 위에 올린다 |
| 테이블 | Cafe table | [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/Cafe%20table) | OpenRobotics | CC0 | DAE 그대로 | 68 | |
| 의자 | Dining Chair | [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/Dining%20Chair) | OpenRobotics | CC-BY 4.0 | DAE 그대로 | 724 | 파일 5 MB. 무거우면 WoodenChair(CC0, OBJ, 면 177) |
| 손님 | Male visitor | [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/Male%20visitor) | OpenRobotics | CC0 | DAE 그대로 | 1,470 | 이미 `dynamic_worker`가 쓴다 |
| 손님 (다양하게) | FemaleVisitor | [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/FemaleVisitor) | OpenRobotics | CC-BY 4.0 | OBJ 그대로 | 1,880 | |

- 면 수: Fuel은 받은 메시에서 직접 셌고, Sketchfab은 사이트 표기 값이다.
- 모두 상업 이용 가능(CC0·CC-BY). NonCommercial·NoDerivatives 라이선스는 뺐다.

## 2. 넣을 때 주의

- **형식**: Gazebo Fortress(ign-common4)는 DAE·OBJ·STL만 읽는다. Sketchfab 모델은 glTF로 받아지므로 Blender에서 OBJ(또는 DAE)로 내보낸다. 텍스처 PNG는 메시와 같은 폴더에 둔다.
- **크기·축**: Sketchfab 모델은 작성자마다 단위와 위쪽 축이 다르다. 변환할 때 실제 크기로 맞추고 Z가 위가 되게 한다. 기준: 배식대 높이 약 0.9 m, 테이블 0.75 m, 카트 약 0.9 m.
- **충돌체**: 배식대·반납대·테이블 모두 바닥까지 꽉 찬 박스로 둔다. 로봇 라이다가 테이블 다리 사이를 빈 곳으로 보지 않게 하기 위해서다(D2).
- **조합**: 배식대는 받침 + 위 모델, 반납대는 카트 + 건조대를 한 모델(`model.sdf`)의 visual 두 개로 묶으면 배치가 쉽다.
- **무게**: 제일 무거운 카트도 면 2만 개 이하다. 반납대는 1~2개, 배식대는 1줄이면 RTF 영향은 작을 것으로 본다. 넣은 뒤 하연의 5대 실행에서 RTF를 확인한다.

## 3. 출처 표기 (CC-BY, 영상 끝·PPT에 넣는다)

```
3D models (CC BY 4.0):
"Buffet" by nicolas saude, "Buffets (food warmer), Foods, and Plates" by sakigakefuruzawa,
"IKEA KUNGSFORS kitchen trolley" by WINTER, "Dish Rack with Dishes" by HippoStance (Sketchfab);
"Dining Chair", "FemaleVisitor" by OpenRobotics (Gazebo Fuel)
```

대안(cafeteria food warmer)이나 WoodenChair로 바꾸면 이 목록도 고친다.

## 4. 할 일

| 할 일 | 담당 |
| --- | --- |
| Sketchfab 5개 내려받기 (로그인 필요) | 민지 |
| OBJ 변환, 크기·축 맞추기, Gazebo에 띄워 확인 | 민지 |
| 식당 월드에 배치, 충돌 박스 | 지수 |

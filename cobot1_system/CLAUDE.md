# cobot1_system

협동로봇1(dsr01, m0609) 피킹 시스템의 ROS2 노드 그래프. 노드 구조도(camera_node /
central_node / robot_arm_node / db_node + ui_node)를 그대로 구현한 패키지다.

## 노드 구성

| 노드 | 역할 | 통신 |
|---|---|---|
| `camera_node` | YOLO(ultralytics)로 물건 인식 | `/image_raw` 퍼블리시, `/detect_object` 서비스 서버 |
| `central_node` | 작업 총괄 | `/start_task` 서비스 서버, `/central_status` 퍼블리시, `/pick` 액션 클라이언트 |
| `robot_arm_node` | 피킹 액션서버 | `/pick` 액션 서버, `/detect_object`·`/update_inventory` 서비스 클라이언트 |
| `db_node` | 재고·로그 | `/update_inventory` 서비스 서버 (in-memory) |
| `ui_node` | 버튼/영상 UI | (팀원 구현 — 아래 인터페이스 계약 참고) |

모든 노드가 `/emergency_stop` (std_msgs/Bool)을 구독한다.

## custom_interfaces

이 패키지가 의존하는 `../custom_interfaces`에 다음을 손봤다:
- `CMakeLists.txt`에 `rosidl_generate_interfaces()` 호출이 없어서 srv/action이 빌드가
  안 되고 있던 걸 추가함.
- `DetectObject.srv` 응답에 `string label` 필드 추가 (YOLO 클래스명을
  `update_inventory`/액션 feedback의 `current_object`까지 전달하기 위함).

## ui_node 인터페이스 계약 (팀원용)

- `/start_task` (custom_interfaces/srv/StartTask): 빈 요청을 보내면 작업 시작.
  응답 `success`(bool)로 실제 시작 여부, 실패 시 `message`(string)에 이유
  (`이미 작업이 진행 중입니다.` / `robot_arm_node 응답 없음`)
- `/central_status` (std_msgs/String): `IDLE` / `RUNNING: ...` / `DONE: total_moved=N` /
  `EMERGENCY_STOP` / `STOPPED: ...` 구독해서 화면 표시
- `/image_raw` (sensor_msgs/Image): 카메라 영상 구독
- `/emergency_stop` (std_msgs/Bool): `true` 발행 시 전체 작업 중단

## robot_arm_node.py — 콜백 체인 구조

`execute_callback`(goal 진입점)은 다음 순서로 콜백을 체인한다 (동기 폴링 없음,
전부 `call_async` + `add_done_callback`):

1. `execute_callback` — `robot_motion.move_to_ready()`로 prime_pos 이동 후
   `_request_detect()` 호출, 이후 `_done_event.wait()`로 전체 goal이 끝날 때까지 블로킹
2. `_request_detect` → `/detect_object` 서비스 호출 → 응답 콜백 `_on_detect_response`
3. `_on_detect_response` — 물건을 찾으면 `_start_grasp` 호출, 없으면 goal 종료
4. `_start_grasp` — **로봇팔 이동 함수 연결 지점**. `robot_motion.pick_object()` 호출
   결과로 성공/재시도(`MAX_RETRY`=3)/포기 후 다음 물건으로 분기. **파지 성공
   (EVENT_GRABBED) 시점에 `_request_inventory_update()`로 재고를 바로 차감**한다
   (바구니에서 들어올린 순간 재고가 빠졌다고 보기 때문 — 이후 낙하해도 재고는
   되돌리지 않음)
5. `_on_inventory_response` → `_place_object()` 호출
6. `_place_object` — `robot_motion.place_object()` 성공 시 `EVENT_PLACED` +
   `total_moved` 증가, 실패(이동 중 낙하) 시 `EVENT_DROPPED`만 보내고 재고는 그대로
   둠. 둘 다 끝나면 다음 물건 인식(`_request_detect`)으로 이어짐. `central_node`가
   `EVENT_DROPPED`를 받으면 `/central_status`에 "컨베이어 벨트로 옮겨주세요" 안내를
   별도로 띄운다(`central_node._feedback_callback`)

## robot_motion.py — 두산 로봇 모션 연동 지점

`robot_arm_node.py`는 실제 로봇 모션을 직접 다루지 않고 `robot_motion.py`에 위임한다.
**물건 잡는 동작 함수는 이 파일만 채우면 된다:**

- `connect()` — DSR_ROBOT2 연동 (완료됨, `dsr01`/`m0609`, `set_tool`/`set_tcp` 설정 포함)
- `move_to_ready()` — **TODO**: prime_pos(대기 자세) 이동. goal 시작 시 호출됨
- `pick_object(center_x, center_y, angle) -> bool` — **TODO**: `cobot1/block1_1.py`의
  movel/task_compliance_ctrl/set_desired_force/get_tool_force 로직을 옮겨서 구현.
  `robot_arm_node._start_grasp()`에서 호출됨
- `place_object() -> bool` — **TODO**: 컨베이어로 이동 후 grip_open
- `grip_close()` / `grip_open()` — 구현됨 (디지털 출력 1/2번)

`pick_object`가 `False`를 반환하면 액션서버가 최대 3회 재시도(`EVENT_RETRY`)한다.
재고 차감은 `pick_object`가 `True`(파지 성공)를 반환한 시점에 이미 끝나므로,
`place_object`가 `False`(이동 중 낙하)를 반환해도 재고는 되돌리지 않는다 — 대신
`EVENT_DROPPED` 피드백이 `central_node`를 거쳐 UI에 "컨베이어 벨트로 옮겨주세요"
안내로 전달된다.

### 주의: 노드 분리 이유
DSR_ROBOT2의 모션 함수는 내부적으로 `rclpy.spin_until_future_complete(node, future)`를
호출하는데, 이 호출은 그 노드를 매번 자기 executor로 옮긴다. 그래서 `robot_motion.py`는
액션서버를 돌리는 `robot_arm_node`와는 별개의 전용 노드(`robot_motion_dsr`)를 만들어
쓴다 — 같은 노드를 쓰면 모션 함수 한 번만 호출해도 액션서버의
MultiThreadedExecutor에서 떨어져 나가 버린다.

### 실제 로봇 없이 테스트할 때
`robot_arm_node`는 시작 시 `connect()`에서 `set_tool` 서비스 응답을 기다리며
블로킹된다. 두산 드라이버가 떠 있지 않으면 그대로 대기 상태가 된다 (정상,
`cobot1`의 다른 스크립트들도 동일하게 동작). 통신 구조만 테스트하려면
`/detect_object`를 흉내내는 가짜 서비스 노드를 띄워서 `central_node`/`db_node`/
`robot_arm_node`를 같이 돌려보면 된다(실 로봇 연결 시에는 `connect()`가 즉시 통과됨).

## 빌드 & 실행

```bash
cd ~/ws_cobot_pjt/ws_edu
colcon build --packages-select custom_interfaces cobot1_system --symlink-install
source install/setup.bash

ros2 run cobot1_system camera_node --ros-args -p camera_index:=0 -p model_path:=yolov8n.pt
ros2 run cobot1_system db_node
ros2 run cobot1_system robot_arm_node
ros2 run cobot1_system central_node
```

import DR_init
import rclpy
from rclpy.node import Node
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            get_tool_force,
            get_current_posx,
            set_tool,
            set_tcp,
            set_digital_output,
            movej,
            movel,
            wait,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # from your_package.srv import DetectObject           # 패키지 이름 정하기

    global prime_pose
    prime_pose = posx([455.00, 240.00, 300.00, 40.00, -180.00, 40.00])  # 초기 위치 좌표
    Approoch_pose = 70.00  # 카메라와 그리퍼의 거리 차이
    conv_pose = posx(
        [278.00, -243.00, 250.00, 40.00, -180.00, 40.00 + 90.00]
    )  # 컨베이어 위치 좌표
    floor_z = 7.00

    set_tool("Tool Weight1")
    set_tcp("GripperDA_v1")

    # ------------------------- YOLO service client -------------------------

    # detect_client = node.create_client(DetectObject, '/detect_object')

    # def request_detection():
    #     """YOLO 서비스에 인식 요청하고 응답(좌표) 받기"""
    #     while not detect_client.wait_for_service(timeout_sec=1.0):
    #         node.get_logger().info('YOLO 서비스 서버 대기 중...')

    #     req = DetectObject.Request()
    #     req.trigger = True

    #     future = detect_client.call_async(req)
    #     rclpy.spin_until_future_complete(node, future)
    #     return future.result()

    # ------------------------- 로봇 구동 함수 -------------------------
    def move_target_pose(response):  # 초기 위치에서 물건으로 이동하는 함수
        obj_pose = posx(
            response.x,  # 입력 받은 x좌표
            response.y - Approoch_pose,  # 입력 받은 y좌표
            prime_pose[2],
            prime_pose[3],
            prime_pose[4],
            prime_pose[5],
        )
        movel(obj_pose, vel=VELOCITY, acc=ACC)  # 물건으로 이동
        return obj_pose  # 물건 위치 반환

    def grip_open():  # 로봇 그리퍼 열기
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(1.0)

    def grip_close():  # 로봇 그리퍼 닫기
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(1.0)

    def grip_checking():  # 그리퍼 잡기 성공 여부 확인
        a = False
        force1 = get_tool_force(DR_BASE)

        if force1[2] < -13:
            a = True
        else:
            a = False
        return a

    def force_down(x):  # 힘 제어 하강 함수
        # 목표 위치로 이동
        movel(x, vel=VELOCITY, acc=ACC)

        # 순응 제어 시작
        task_compliance_ctrl(stx=[3000, 3000, 500, 300, 300, 300])
        time.sleep(0.5)

        # Z축 -30N 힘을 가하며 하강
        set_desired_force(
            fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL
        )
        time.sleep(0.5)

        # 외력 감지 (z축 힘 >= 5N이면 물체에 닿은 것)
        while True:
            force_ext = get_tool_force(DR_BASE)
            print(f"force_ext = {force_ext}")
            if force_ext[2] >= 5:
                break
            time.sleep(0.5)

        # 5. 힘 제어 해제
        release_force()
        time.sleep(0.5)

        # 6. 순응 제어 해제
        release_compliance_ctrl()
        time.sleep(0.5)

    def pick_up(max_try=5):  # z축 높이 차이에 따라 그리퍼 위치 조절 후 잡기
        cur_pos = get_current_posx()[0]
        z_diff = cur_pos[2] - floor_z
        print(f"z_diff = {z_diff}, cur_z = {cur_pos[2]}")

        # 1. 살짝 위로 (여유 공간)
        cur_pos[2] += 5
        movel(cur_pos, vel=50, acc=70)

        # 2. 물체 높이에 따라 내려갈 거리 결정
        if 23 <= z_diff < 43:
            down = 25
        elif z_diff >= 43:
            down = 35
        else:
            print(f"z_diff({z_diff})가 23 미만 → 잡기 불가")
            return False

        # z 위치 미리 저장 (누적 방지)
        up_z = cur_pos[2]  # 위 위치 (올라온 곳)
        grip_z = cur_pos[2] - down  # 아래 위치 (잡는 곳)

        for attempt in range(1, max_try + 1):
            print(f"잡기 시도 {attempt}/{max_try}")

            # 3. 그리퍼 열고 내려가서 잡기
            grip_open()
            wait(2.0)

            cur_pos[2] = grip_z  # 항상 같은 높이로 내림
            movel(cur_pos, vel=50, acc=70)
            wait(0.5)

            grip_close()
            wait(2.0)

            # 4. 살짝 들어올린 후 잡기 판단
            cur_pos[2] = up_z  # 위로 올림
            movel(cur_pos, vel=50, acc=70)
            wait(0.5)

            if grip_checking():  # 들어올린 상태에서 힘 체크
                print("잡기 성공!")
                break
            else:
                print(f"잡기 실패 ({attempt}/{max_try})")
        else:
            print(f"{max_try}회 실패 → 방해물로 판단, 건너뜀")
            return False

        # 5. 들어올리기
        cur_pos[2] += 200
        movel(cur_pos, vel=50, acc=70)

        return True

    def move_to_conv():  # 컨베이어 위치로 이동 후 물건 놓기
        # 컨베이어 위치로 이동
        movel(conv_pose, vel=VELOCITY, acc=ACC)

        # 컨베이어에 물건 놓기
        force_down(conv_pose)
        grip_open()

        # 초기 위치로 이동
        movel(prime_pose, vel=VELOCITY, acc=ACC)

    # ------------------------- 메인 프로세스 -------------------------
    movel(prime_pose, vel=VELOCITY, acc=ACC)  # 초기 위치로

    while rclpy.ok():
        response = request_detection()  # YOLO 인식 요청

        if not response.success:  # 물건 없으면 종료
            print("장바구니 비어있음 → 작업 완료")
            break

        obj_pose = move_target_pose(response)  # xy 이동 + 좌표 받음
        force_down(obj_pose)  # 하강 + 접촉 감지
        pick_up()  # 잡기
        move_to_conv()  # 컨베이어 이동 + prime_pose 복귀

        print("물건 1개 처리 완료")

    rclpy.shutdown()


if __name__ == "__main__":
    main()

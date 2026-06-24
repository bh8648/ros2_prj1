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
            get_curerent_posx,
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
    Approoch_pose = 75.00  # 카메라와 그리퍼의 거리 차이
    conv_pose = posx([])  # 컨베이어 위치 좌표

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
    def move_target_pose(response):  # 초기 위치에서 첫 번째 물건으로 이동하는 함수
        obj_pose = posx(
            response.x,  # 입력 받은 x좌표
            response.y - Approoch_pose,  # 입력 받은 y좌표
            prime_pose[2],
            prime_pose[3],
            prime_pose[4],
            prime_pose[5],
        )
        movel(obj_pose, vel=VELOCITY, acc=ACC)  # 첫 번째 물건으로 이동

    def grip_open():  # 로봇 그리퍼 열기
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(1.0)

    def grip_close():  # 로봇 그리퍼 닫기
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(1.0)

    def grip_checking():
        a = False
        force1 = get_tool_force(DR_BASE)

        if force1[2] < -13:
            a = True
        else:
            a = False
        return a

    def force_down(x):
        # 그립 클로즈 기준으로 동작
        movel(x, vel=VELOCITY, acc=ACC)
        task_compliance_ctrl(stx=[3000, 3000, 500, 300, 300, 400])
        time.sleep(0.5)

        set_desired_force(
            fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL
        )

        # 외력이 0 이상 5 이하이면 0
        # 외력이 5 초과이면 -1
        while not check_force_condition(DR_AXIS_Z, max=5):
            time.sleep(0.5)

        release_force()  # 힘 제어 해제
        time.sleep(0.5)

        release_compliance_ctrl()  # 순응 제어 해제

    def move_to_conv():  # 컨베이어에 물건 놓고 초기 위치 복귀
        # 컨베이어 위치로 이동
        movel(conv_pose, vel=VELOCITY, acc=ACC)

        # 컨베이어에 물건 놓기
        force_down(conv_pose)
        grip_open()

        # 초기 위치로 이동
        movel(prime_pose, vel=VELOCITY, acc=ACC)


if __name__ == "__main__":
    main()

# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy

# =========================
# 로봇 기본 설정
# =========================

# 사용할 로봇 ID
ROBOT_ID = "dsr01"

# 사용할 로봇 모델명
ROBOT_MODEL = "m0609"

# 기본 속도 및 가속도 설정
VELOCITY, ACC = 30, 30

import DR_init

# DSR 초기화 모듈에 로봇 정보 등록
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # 노드 생성
    node = rclpy.create_node(
        "rokey_move_periodic",
        namespace=ROBOT_ID
    )

    # 생성한 노드를 DSR 라이브러리에 등록
    DR_init.__dsr__node = node

    try:
        # 두산 로봇 제어 함수 import
        from DSR_ROBOT2 import (
            amove_periodic,  # 비동기 주기운동 명령
            set_tool,        # 툴 설정
            set_tcp,         # TCP 설정
            movej,           # Joint 이동
            DR_TOOL,         # 툴 기준 좌표계
        )

        # Joint Position 생성 함수
        from DR_common2 import posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # =========================
    # Tool / TCP 설정
    # =========================

    # 로봇에 등록된 툴 정보 적용
    set_tool("Tool Weight_RG2")

    # TCP(Tool Center Point) 설정
    set_tcp("RG2_TCP")

    # =========================
    # 초기 대기 자세 정의
    # =========================

    # Joint 자세 정의
    # [J1, J2, J3, J4, J5, J6]
    JReady = posj([0, 0, 90, 0, 90, 0])

    # =========================
    # 주기운동(Periodic Motion) 설정
    # =========================

    # 각 축별 진폭(amplitude)
    # 마지막 J6 축만 ±30도 진동
    example_amp = [0.0, 0.0, 0.0, 0.0, 0.0, 30.0]

    # ROS2 상태 확인
    if rclpy.ok():

        # Ready 자세로 이동
        print(f"Moving to joint position: {JReady}")

        # Joint Move 수행
        movej(
            JReady,
            vel=VELOCITY,
            acc=ACC
        )

        # 주기운동 시작
        print(f"Starting amove_periodic: {example_amp}")

        # 비동기 주기운동 실행
        #
        # amp    : 진폭
        # period : 1회 왕복 주기(초)
        # atime  : 가감속 시간(초)
        # repeat : 반복 횟수
        # ref    : 기준 좌표계
        #

        
        amove_periodic(
            amp=example_amp,
            period=1.0,
            atime=0.02,
            repeat=3,
            ref=DR_TOOL
        )

    # ROS2 종료
    rclpy.shutdown()


if __name__ == "__main__":
    main()
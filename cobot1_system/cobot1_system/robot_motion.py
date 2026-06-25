"""두산 로봇(DSR_ROBOT2) 모션 연동 모듈.

robot_arm_node가 호출하는 픽&플레이스 함수의 틀(스텁)이다. 실제 동작
(이미지->로봇 좌표 변환, movel/movej, 그리퍼 제어, 힘제어 등)은 이 파일의
TODO 부분만 채우면 robot_arm_node 쪽 코드는 손댈 필요가 없다.
cobot1/block1_1.py 에 있던 grap_open/grap_close, movel, task_compliance_ctrl,
set_desired_force, get_tool_force 로직을 그대로 옮겨서 구현하면 된다.
"""
import rclpy
import DR_init

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

dsr = None    # connect() 호출 후 DSR_ROBOT2 모듈이 채워진다.
_node = None  # DSR_ROBOT2 전용 노드


def connect():
    """DSR_ROBOT2 전용 노드를 새로 만들어 두산 로봇에 연결한다.

    rclpy.init() 이후 RobotArmNode 생성 시 한 번만 호출해야 한다.

    주의: 이 노드를 robot_arm_node의 MultiThreadedExecutor에 절대 추가하면
    안 된다. DSR_ROBOT2의 모션 함수들은 내부적으로
    rclpy.spin_until_future_complete(node, future)를 호출하는데, 이 호출은
    매번 해당 노드의 executor를 자기 자신(전역 기본 executor)으로 바꿔버린다.
    그래서 액션서버 콜백을 처리하는 노드와 DSR_ROBOT2가 붙는 노드는 반드시
    분리해야 한다 (안 그러면 모션 함수 한 번만 호출해도 액션서버가 멈춘다).
    """
    global dsr, _node
    _node = rclpy.create_node('robot_motion_dsr', namespace=ROBOT_ID)
    DR_init.__dsr__node = _node

    import DSR_ROBOT2 as dsr_module
    dsr = dsr_module

    dsr.set_tool('Tool Weight_2FG')
    dsr.set_tcp('2FG_TCP')
    _node.get_logger().info('DSR_ROBOT2 연동 완료 (dsr01 / m0609)')


def shutdown():
    if _node is not None:
        _node.destroy_node()


def move_to_ready():
    """대기 자세로 이동.

    TODO: 팀에서 실제 Ready 자세(JReady)를 정의하고 movej로 이동.
    """


def pick_object(center_x: float, center_y: float, angle: float) -> bool:
    """카메라가 인식한 물건(center_x, center_y, angle)을 집는다.

    Args:
        center_x, center_y: 이미지 좌표계 중심 (px). 로봇 베이스 좌표로 변환 필요.
        angle: 물건의 기울기 (deg).

    Returns:
        성공적으로 집었으면 True, 놓쳤으면 False.
        False를 반환하면 robot_arm_node가 EVENT_RETRY 피드백을 보내고
        같은 위치를 다시 인식해서 재시도한다.

    TODO: block1_1.py 로직을 아래 순서로 옮겨서 구현.
        1) 이미지 좌표 -> 로봇 베이스 좌표 변환 (캘리브레이션 행렬 필요)
        2) movel(target_pos, VELOCITY, ACC) 로 접근
        3) grip_close() 로 파지
        4) task_compliance_ctrl + set_desired_force + get_tool_force 로 파지 확인
    """
    return True


def place_object() -> bool:
    """집은 물건을 목적지(컨베이어 등)에 내려놓는다.

    Returns:
        정상적으로 내려놓았으면 True. False면 이동 중 낙하한 것으로 보고
        robot_arm_node가 EVENT_DROPPED 피드백을 보낸다.

    TODO: movel(place_pos, VELOCITY, ACC) 로 이동 후 grip_open()으로 내려놓기.
    """
    return True


def grip_close():
    dsr.set_digital_output(2, dsr.ON)
    dsr.set_digital_output(1, dsr.OFF)
    dsr.wait(1.0)


def grip_open():
    dsr.set_digital_output(1, dsr.ON)
    dsr.set_digital_output(2, dsr.OFF)
    dsr.wait(1.0)

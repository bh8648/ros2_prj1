"""두산 로봇(DSR_ROBOT2) 모션 연동 모듈.

robot_arm_node가 호출하는 픽&플레이스 함수 구현. robot_arm_node는 이 파일의
pick_object()/place_object()/move_to_ready()만 호출한다.
"""
import time
import numpy as np
import rclpy
import DR_init

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
VELOCITY, ACC = 50, 50

ON, OFF = 1, 0

floor_z = 7.00

# 비상정지 플래그. True가 되면 connect()에서 감싼 movel/movej가 무효화되고
# 모션 루프들이 빠져나와 로봇이 더 이상 움직이지 않는다. 시작(goal) 시 해제된다.
_emergency = False


def request_stop():
    """비상정지: 이후 모든 모션을 무효화한다."""
    global _emergency
    _emergency = True


def clear_stop():
    """비상정지 해제: 모션을 다시 허용한다."""
    global _emergency
    _emergency = False


def is_stopped():
    return _emergency


def _wrap_motion_guard():
    """dsr.movel/movej를 비상정지 플래그를 확인하는 버전으로 감싼다.

    _emergency가 True이면 모든 movel/movej 호출이 즉시 no-op이 되어, 비상정지
    후에는 어떤 코드 경로에서도 로봇이 움직이지 않는다.
    """
    _orig_movel = dsr.movel
    _orig_movej = dsr.movej

    def movel_guarded(*args, **kwargs):
        if _emergency:
            return None
        return _orig_movel(*args, **kwargs)

    def movej_guarded(*args, **kwargs):
        if _emergency:
            return None
        return _orig_movej(*args, **kwargs)

    dsr.movel = movel_guarded
    dsr.movej = movej_guarded


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
dsr = None          # connect() 호출 후 DSR_ROBOT2 모듈이 채워진다.
_node = None        # DSR_ROBOT2 전용 노드
prime_posj = None   # 초기 대기 관절자세(posj) — connect()에서 채워진다.
conv_pose = None    # 컨베이어 위치 좌표 — connect()에서 posx로 채워진다.

top_left = [292.00,292.00]
top_right = [610.00, 255.00]
bot_left = [283.00, -64.00]
bot_right = [611.00, -63.00]

# ============================================================================
# 카메라 픽셀 좌표 -> 로봇 베이스 평면 좌표(mm) 변환 캘리브레이션.
#
# ★ 장바구니 모서리를 다시 잴 때는 아래 _px 4줄만 바꾸면 된다.
#   (호모그래피 행렬 IMG2ROBOT_H는 자동으로 다시 계산됨)
#
#   1) tools/calibrate_corners.py 실행 → 모서리 클릭 → 픽셀값 출력
#   2) 출력된 픽셀값을 아래 top_left_px ~ bot_right_px 에 그대로 입력
#   3) colcon build 후 노드 재시작
#
# 로봇 좌표(top_left ~ bot_right, 위에 정의)는 장바구니 물리 위치가 바뀌지
# 않는 한 그대로 둔다. 픽셀↔로봇은 같은 모서리끼리 같은 순서로 대응돼야 한다.
# ============================================================================
top_left_px = [504, 439]
top_right_px = [82, 450]
bot_left_px = [479, 34]
bot_right_px = [84, 26]


def _compute_homography():


    src = [top_left_px, top_right_px, bot_left_px, bot_right_px]
    dst = [top_left, top_right, bot_left, bot_right]
    A, b = [], []
    for (x, y), (X, Y) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -x * X, -y * X])
        A.append([0, 0, 0, x, y, 1, -x * Y, -y * Y])
        b += [X, Y]
    h = np.linalg.solve(np.array(A, float), np.array(b, float))
    return [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]]


IMG2ROBOT_H = _compute_homography()

# 호모그래피 후 미세 보정 오프셋(mm). 카메라 광심과 그리퍼 TCP 장착 차이로
# 생기는 일정한 편차를 잡는다. 실측에서 TCP가 물건보다 +x로 ~40mm 치우쳐
# OFFSET_X로 당긴다. TCP가 반대로 틀어지면 부호를 뒤집고, 위치마다 편차가
# 다르면 상수 보정이 아니라 캘리브레이션 재측정이 필요하다.
OFFSET_X = 0.0
OFFSET_Y = 0.0


def image_to_robot(px, py):
    """카메라 픽셀 좌표(px, py)를 로봇 베이스 평면 좌표(mm)로 변환한다."""
    h = IMG2ROBOT_H
    w = h[2][0] * px + h[2][1] * py + h[2][2]
    rx = (h[0][0] * px + h[0][1] * py + h[0][2]) / w
    ry = (h[1][0] * px + h[1][1] * py + h[1][2]) / w
    return rx + OFFSET_X, ry + OFFSET_Y

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
    global dsr, _node, prime_pose, prime_posj, conv_pose
    _node = rclpy.create_node('robot_motion_dsr', namespace=ROBOT_ID)
    DR_init.__dsr__node = _node

    import DSR_ROBOT2 as dsr_module
    dsr = dsr_module

    # 비상정지 시 모든 movel/movej를 무효화하도록 감싼다.
    _wrap_motion_guard()

    dsr.set_tool("Tool_Weight1")
    dsr.set_tcp("GripperDA_v1")

    dsr.DR_BASE = 0
    dsr.DR_TOOL = 1

    # posx는 dsr 연동 후에만 만들 수 있으므로 여기서 초기화한다.
    prime_pose = dsr.posx([455.00, 240.00, 300.00, 40.00, -180.00, 40.00])
    prime_posj = dsr.posj([25.85, 30.91, 36.05, 0.17, 113.05, 31.97])
    conv_pose = dsr.posx([278.00, -243.00, 250.00, 40.00, -180.00, 40.00 + 90.00])

    _node.get_logger().info('DSR_ROBOT2 연동 완료 (dsr01 / m0609)')

def move_target_pose(robot_x, robot_y):  # 로봇 평면 좌표로 물건 위로 이동
        dsr.wait(0.5)
        grip_close()
        dsr.wait(0.5)
        obj_pose = dsr.posx(
            robot_x,
            robot_y,
            prime_pose[2],
            prime_pose[3],
            prime_pose[4],
            prime_pose[5],
        )
        dsr.movel(obj_pose, vel=VELOCITY, acc=ACC)  # 물건으로 이동
        return obj_pose  # 물건 위치 반환


def force_down():  # 힘 제어 하강 함수
    # 목표 위치로 이동

    # 순응 제어 시작
    dsr.task_compliance_ctrl(stx=[3000, 3000, 500, 300, 300, 300])
    time.sleep(0.5)

    # Z축 -30N 힘을 가하며 하강
    dsr.set_desired_force(
        fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=dsr.DR_FC_MOD_REL
    )
    time.sleep(0.5)

    # 외력 감지 (z축 힘 >= 5N이면 물체에 닿은 것)
    while True:
        if _emergency:  # 비상정지: 하강 대기 루프 탈출
            break
        force_ext = dsr.get_tool_force(dsr.DR_BASE)
        print(f"force_ext = {force_ext}")
        if force_ext[2] >= 6:
            break
        time.sleep(0.5)

    # 5. 힘 제어 해제
    dsr.release_force()
    time.sleep(0.5)

    # 6. 순응 제어 해제
    dsr.release_compliance_ctrl()
    time.sleep(0.5)


def press_move():
    pose_press_init = dsr.get_current_posx()[0]
    dsr.task_compliance_ctrl(stx=[3000, 3000, 500, 300, 300, 300])
    time.sleep(0.5)


    dsr.set_desired_force(
        fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=dsr.DR_FC_MOD_REL
    )
    time.sleep(0.5)
    
    dsr.movel([prime_pose[0], pose_press_init[1], pose_press_init[2], prime_pose[3], prime_pose[4], prime_pose[5]], VELOCITY, ACC, ref = dsr.DR_BASE)

    dsr.release_force()
    time.sleep(0.5)

    # 6. 순응 제어 해제
    dsr.release_compliance_ctrl()
    time.sleep(0.5)


    

    

def pick_up(max_try=5):  # z축 높이 차이에 따라 그리퍼 위치 조절 후 잡기
    cur_pos = dsr.get_current_posx()[0]
    z_diff = cur_pos[2] - floor_z
    print(f"z_diff = {z_diff}, cur_z = {cur_pos[2]}")

    # 1. 살짝 위로 (여유 공간)
    cur_pos[2] += 5.00
    dsr.movel(cur_pos, vel=50, acc=70)

    # 2. 물체 높이에 따라 내려갈 거리 결정
    if 23.00 <= z_diff < 43.00:
        down = 35.00
    elif z_diff >= 43.00:
        down = 50.00
    else:
        print(f"z_diff({z_diff})가 23 미만 → 잡기 불가")
        return False

    # z 위치 미리 저장 (누적 방지)
    up_z = cur_pos[2]  # 위 위치 (올라온 곳)
    grip_z = cur_pos[2] - down  # 아래 위치 (잡는 곳)

    for attempt in range(1, max_try + 1):
        if _emergency:  # 비상정지: 파지 재시도 루프 탈출
            return False
        print(f"잡기 시도 {attempt}/{max_try}")

        # 3. 그리퍼 열고 내려가서 잡기
        grip_open()
        dsr.wait(2.0)

        cur_pos[2] = grip_z  # 항상 같은 높이로 내림
        dsr.movel(cur_pos, vel=50, acc=70)
        dsr.wait(0.5)

        grip_close()
        dsr.wait(2.0)

        # 4. 살짝 들어올린 후 잡기 판단
        cur_pos[2] = up_z  # 위로 올림
        dsr.movel(cur_pos, vel=50, acc=70)
        dsr.wait(0.5)

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
    dsr.movel(cur_pos, vel=50, acc=70)

    return True

def move_to_conv():  # 컨베이어 위치로 이동 후 물건 놓기. 낙하 시 False 반환
    # 컨베이어 위치로 이동
    dsr.movel(conv_pose, vel=VELOCITY, acc=ACC)

    # 이동 중 낙하 감지: 도착했는데 물건이 잡혀있지 않으면 떨어뜨린 것
    if not grip_checking():
        grip_open()
        dsr.movel(prime_pose, vel=VELOCITY, acc=ACC)
        return False

    # 컨베이어에 물건 놓기
    force_down()
    grip_open()
    dsr.movel([0, 0, -150, 0, 0, 0], VELOCITY, ACC, ref=dsr.DR_TOOL)
    # 초기 위치로 이동
    dsr.movel(prime_pose, vel=VELOCITY, acc=ACC)
    return True

def shutdown():
    if _node is not None:
        _node.destroy_node()


def move_to_ready():
    grip_close()
    # dsr.movel(prime_pose, VELOCITY, ACC, ref=0)
    dsr.movej(prime_posj, VELOCITY, ACC)


def grip_checking():  # 그리퍼 잡기 성공 여부 확인
    a = False
    force1 = dsr.get_tool_force(dsr.DR_BASE)

    if force1[2] < 5:
        a = True
    else:
        a = False
    return a


def grapping_side_object_left(angle):  # 물건의 x좌표가 370 이하일때 쓰는 grapping 함수

        time.sleep(0.5)

        dsr.movel([0, 0, -10, 0, 0, 0], VELOCITY, ACC, ref=dsr.DR_TOOL)
        dsr.wait(0.5)

        dsr.movel([0, 0, 0, 0, 45, 0], VELOCITY, ACC, ref=dsr.DR_TOOL)
        dsr.wait(0.5)
        

        dsr.movel([0, 0, 0, 0, 0, 90], VELOCITY, ACC, ref = dsr.DR_TOOL)

        grip_open()

        pose2 = dsr.get_current_posx()[0]
        pose2[2] /= 2
        dsr.movel(pose2, VELOCITY, ACC, ref=dsr.DR_BASE)
        pose3 = dsr.get_current_posx()[0]
        pose3[0] -= 30.00
        dsr.movel(pose3, VELOCITY, ACC, ref = dsr.DR_BASE)


        grip_close()
        dsr.wait(1.5)
        # a = dsr.get_current_posx()[0]
        # a[2] += 100
        dsr.movel(prime_pose, VELOCITY, ACC, ref=dsr.DR_BASE)


def pick_object(center_x: float, center_y: float, angle: float) -> bool:
    """카메라가 인식한 물건(center_x, center_y, angle)을 집는다.

    바구니 벽 근처(코너에서 75mm 이내)의 물건은 측면 파지로, 그 외(가운데)
    물건은 angle만큼 그리퍼를 회전한 뒤 일반 탑다운 파지로 처리한다.

    Returns:
        성공적으로 집었으면 True, 놓쳤으면 False. False를 반환하면
        robot_arm_node가 EVENT_RETRY 피드백을 보내고 같은 물건을 재시도한다.
    """
    dsr.wait(0.5)
    grip_close()
    time.sleep(0.5)

    # 카메라가 준 픽셀 좌표를 로봇 평면 좌표로 변환한 뒤, 모든 판정/이동을 로봇 좌표로 한다.
    robot_x, robot_y = image_to_robot(center_x, center_y)

    near_edge = (
        robot_x < top_left[0] + 100.00
        or robot_y < bot_left[1] + 100.00
        or robot_x > top_right[0] - 100.00
        or robot_y > top_right[1] - 100.00
    )

    if near_edge:
        move_target_pose(robot_x, robot_y)
        force_down()

        if dsr.get_current_posx()[0][2] < 70.00:
            press_move()
            success = pick_up()
        else:
            grapping_side_object_left(angle)

            success = grip_checking()
    else:
        move_target_pose(robot_x, robot_y)
        # angle 값이 -90 ~ 90 사이라고 가정 (일자로 배치된 상태를 0도로 본다)
        if angle >= 0:
            dsr.movel([0, 0, 0, 0, 0, angle - 90], VELOCITY, ACC, ref=dsr.DR_TOOL)
        else:
            dsr.movel([0, 0, 0, 0, 0, angle + 90], VELOCITY, ACC, ref=dsr.DR_TOOL)

        dsr.movel([0, 0, 150.00, 0, 0, 0], VELOCITY, ACC, ref=dsr.DR_TOOL)
        force_down()
        success = pick_up()

    return success


def place_object() -> bool:
    """집은 물건을 컨베이어로 옮겨 내려놓고 prime 자세로 복귀한다.

    Returns:
        정상적으로 내려놓았으면 True. 이동 중 낙하했으면 False를 반환해
        robot_arm_node가 EVENT_DROPPED 피드백을 보내게 한다.
    """
    return move_to_conv()


def grip_close():
    dsr.set_digital_output(2, ON)
    dsr.set_digital_output(1, OFF)
    dsr.wait(0.5)


def grip_open():
    dsr.set_digital_output(1, ON)
    dsr.set_digital_output(2, OFF)
    dsr.wait(2.0)

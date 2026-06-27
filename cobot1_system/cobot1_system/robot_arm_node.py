import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from dsr_msgs2.srv import MoveStop

from custom_interfaces.action import PickAndPlace
from custom_interfaces.srv import DetectObject

from . import robot_motion

MAX_RETRY = 3


class RobotArmNode(Node):
    """피킹 액션서버. 실제 픽/플레이스 모션은 robot_motion.py에 위임한다.

    진행 흐름: goal 수신 -> prime_pos 이동 -> /detect_object 서비스 호출(콜백) ->
    응답 수신 시 파지 시작 콜백(_start_grasp) -> 파지 성공 시 재고 차감(콜백) ->
    place(낙하해도 재고는 유지) -> 다음 물건 인식으로 반복. 한 번에 goal 하나만
    처리하므로 진행 상태는 인스턴스 속성에 보관한다.
    """

    def __init__(self):
        super().__init__('robot_arm_node')

        try:
            robot_motion.connect()
        except Exception as e:  # noqa: BLE001 - 로봇 미연결 상태에서도 그래프 테스트 가능하도록
            self.get_logger().error(f'DSR_ROBOT2 연동 실패 (로봇 연결 확인 필요): {e}')

        self.emergency_stopped = False

        # 비상정지는 액션이 모션에 블로킹돼 있어도 즉시 처리돼야 하므로 전용 그룹 사용.
        monitor_cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10,
            callback_group=monitor_cb_group)
        # 컨트롤러 move_stop 서비스로 진행 중인 모션을 물리적으로 즉시 정지한다.
        # (DSR 파이썬 래퍼가 아니라 컨트롤러 노드를 직접 호출 → robot_motion_dsr 미경유)
        self.move_stop_client = self.create_client(
            MoveStop, f'/{robot_motion.ROBOT_ID}/motion/move_stop',
            callback_group=monitor_cb_group)

        # 액션 실행 스레드와 서비스 응답 콜백이 서로 다른 그룹/스레드에서 돌아야
        # execute_callback의 _done_event.wait()가 서비스 콜백을 막지 않는다.
        client_cb_group = ReentrantCallbackGroup()
        action_cb_group = ReentrantCallbackGroup()

        self.detect_client = self.create_client(
            DetectObject, '/detect_object', callback_group=client_cb_group)
       
        # 컨베이어: place(move_to_conv) 완료 후 벨트를 10초 구동시킨다.
        self.conveyor_client = self.create_client(
            Trigger, '/conveyor/run', callback_group=client_cb_group)

        self._action_server = ActionServer(
            self,
            PickAndPlace,
            '/pick',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_cb_group,
        )

        # 현재 처리 중인 goal의 진행 상태
        self._goal_handle = None
        self._feedback_msg = None
        self._current_label_id = -1
        self._current_label = ''
        self._total_moved = 0
        self._retry_count = 0
        self._done_event = None
        self._result = None

        self.get_logger().info('robot_arm_node 시작됨 (피킹 액션서버)')

    def emergency_stop_callback(self, msg):
        self.emergency_stopped = msg.data
        if self.emergency_stopped:
            self.get_logger().warn('긴급정지 신호 수신 - 로봇 즉시 정지')
            robot_motion.request_stop()  # 이후 모든 모션 무효화 (다시 안 움직임)
            self._stop_robot()           # 진행 중 모션 물리 정지
        else:
            robot_motion.clear_stop()

    def _stop_robot(self):
        """컨트롤러 move_stop 서비스로 진행 중인 모션을 즉시 정지(DR_QSTOP)."""
        if self.move_stop_client.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1  # DR_QSTOP: Quick stop (Stop Category 2)
            self.move_stop_client.call_async(req)
        else:
            self.get_logger().warn('move_stop 서비스 미연결 - 물리 정지 생략')

    def goal_callback(self, goal_request):
        # 시작(start) 요청 = 사용자가 다시 작업을 시키는 것이므로, 비상정지 상태를
        # 해제하고 작업을 재개한다. (비상정지 후엔 시작 누를 때까지 아무것도 안 함)
        self.emergency_stopped = False
        robot_motion.clear_stop()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # 액션 진입점: prime_pos 이동 후 첫 인식을 시작하고, 콜백 체인이 끝날 때까지 대기한다.
    # ------------------------------------------------------------------
    def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._feedback_msg = PickAndPlace.Feedback()
        self._total_moved = 0
        self._retry_count = 0
        self._done_event = threading.Event()
        self._result = None

        robot_motion.move_to_ready()  # prime_pos 이동
        self._request_detect()

        self._done_event.wait()
        return self._result

    # ------------------------------------------------------------------
    # 1) 카메라 인식 서비스 요청 (콜백 기반, 폴링하지 않음)
    # ------------------------------------------------------------------
    def _request_detect(self):
        if self._check_abort():
            return

        # 카메라 노드가 없으면 future가 영원히 완료되지 않아 액션이 멈춘다.
        # 서버가 안 떠 있으면 작업을 중단한다.
        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('카메라(/detect_object) 응답 없음 - 작업 중단')
            self._goal_handle.abort()
            self._finish(success=False)
            return

        request = DetectObject.Request()
        request.capture = True
        future = self.detect_client.call_async(request)
        future.add_done_callback(self._on_detect_response)

    def _on_detect_response(self, future):
        response = future.result()
        if response is None or not response.found:
            self._goal_handle.succeed()
            self._finish(success=True)
            return

        self._retry_count = 0
        self._current_label_id = response.label_id
        self._current_label = response.label or 'unknown'
        self._feedback_msg.current_label_id = self._current_label_id
        self._feedback_msg.current_object = self._current_label
        self._feedback_msg.moved_count = self._total_moved
        self._start_grasp(response)

    # ------------------------------------------------------------------
    # 2) 파지 시작 콜백. 실제 로봇팔 이동 함수는 robot_motion.pick_object()에 연결한다
    #    (cobot1/block1_1.py의 movel/grip_close/force 로직을 그 함수에 채우면 됨).
    # ------------------------------------------------------------------
    def _start_grasp(self, detect_response):
        if self._check_abort():
            return

        event = (
            PickAndPlace.Feedback.EVENT_RETRY
            if self._retry_count > 0 else PickAndPlace.Feedback.EVENT_PICKING
        )
        self._feedback_msg.event = event
        self._goal_handle.publish_feedback(self._feedback_msg)

        # TODO: 로봇팔 이동/파지 함수 연결 지점. 지금은 robot_motion.pick_object()가
        # 스텁(항상 True)을 반환한다.
        grabbed = robot_motion.pick_object(
            detect_response.center_x, detect_response.center_y, detect_response.angle)

        # 파지 모션 중 비상정지/취소가 들어왔으면 재시도하지 말고 즉시 종료한다.
        if self._check_abort():
            return

        if grabbed:
            self._feedback_msg.event = PickAndPlace.Feedback.EVENT_GRABBED
            self._goal_handle.publish_feedback(self._feedback_msg)
            
            self._place_object()
            return

        self._retry_count += 1
        if self._retry_count >= MAX_RETRY:
            self.get_logger().warn(f'파지 실패 (재시도 초과): {self._current_label}')
            self._request_detect()  # 다음 물건으로 넘어간다
            return

        self.get_logger().warn(f'파지 실패 ({self._retry_count}/{MAX_RETRY}): {self._current_label}')
        self._start_grasp(detect_response)

    # ------------------------------------------------------------------
    # 3) 플레이스. 컨베이어에 정상적으로 놓이면 PLACED feedback을 보낸다.
    #    재고 차감은 central_node가 PLACED feedback을 받은 뒤 수행한다.
    # ------------------------------------------------------------------
    def _place_object(self):
        if self._check_abort():
            return

        if robot_motion.place_object():
            self._total_moved += 1
            self._feedback_msg.event = PickAndPlace.Feedback.EVENT_PLACED
            self._feedback_msg.moved_count = self._total_moved
            self._goal_handle.publish_feedback(self._feedback_msg)
            # 컨베이어에 안착했으니 벨트를 구동(10초)한 뒤 다음 물건으로 넘어간다.
            self._request_conveyor()
            return

        # 이동 중 낙하: PLACED가 아니므로 재고 차감은 하지 않는다., central_node가 이 이벤트를
        # 받아 UI에 "컨베이어로 옮겨달라" 안내를 띄운다. 벨트는 돌리지 않는다.
        self._feedback_msg.event = PickAndPlace.Feedback.EVENT_DROPPED
        self._goal_handle.publish_feedback(self._feedback_msg)
        self.get_logger().warn(f'이동 중 낙하: {self._current_label}')
        self._request_detect()

    def _request_conveyor(self):
        # 컨베이어 노드가 없어도 작업은 계속한다(벨트 구동만 건너뜀).
        if not self.conveyor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('컨베이어 서비스(/conveyor/run) 없음 - 벨트 구동 건너뜀')
            self._request_detect()
            return
        future = self.conveyor_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_conveyor_response)

    def _on_conveyor_response(self, future):
        response = future.result()
        if response is None or not response.success:
            msg = response.message if response is not None else '응답 없음'
            self.get_logger().warn(f'컨베이어 구동 실패: {msg}')
        else:
            self.get_logger().info('컨베이어 구동 완료')
        self._request_detect()

    # ------------------------------------------------------------------
    def _check_abort(self):
        """긴급정지/취소 상태면 goal을 종료하고 True를 반환한다."""
        if self.emergency_stopped:
            self._goal_handle.abort()
            self._finish(success=False)
            return True
        if self._goal_handle.is_cancel_requested:
            self._goal_handle.canceled()
            self._finish(success=False)
            return True
        return False

    def _finish(self, success):
        result = PickAndPlace.Result()
        result.total_moved = self._total_moved
        result.success = success
        self._result = result
        self._done_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot_motion.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

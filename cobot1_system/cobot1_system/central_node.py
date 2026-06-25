import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from custom_interfaces.action import PickAndPlace
from custom_interfaces.srv import StartTask

_EVENT_NAMES = {
    PickAndPlace.Feedback.EVENT_PICKING: 'PICKING',
    PickAndPlace.Feedback.EVENT_GRABBED: 'GRABBED',
    PickAndPlace.Feedback.EVENT_PLACED: 'PLACED',
    PickAndPlace.Feedback.EVENT_RETRY: 'RETRY',
    PickAndPlace.Feedback.EVENT_DROPPED: 'DROPPED',
}


class CentralNode(Node):
    """작업 총괄 노드. ui_node의 버튼 입력을 받아 robot_arm_node에 피킹 액션을 요청한다."""

    def __init__(self):
        super().__init__('central_node')

        self._goal_handle = None
        self._busy = False                      # 작업 진행중이면 True

        self._action_client = ActionClient(self, PickAndPlace, '/pick')

        self.status_pub = self.create_publisher(String, '/central_status', 10)
        self.create_service(StartTask, '/start_task', self.start_task_callback)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        self._publish_status('IDLE')
        self.get_logger().info('central_node 시작됨 (작업 총괄)')

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'상태: {text}')

    def start_task_callback(self, request, response):
        if self._busy:
            response.success = False
            response.message = '이미 작업이 진행 중입니다.'
            self.get_logger().warn(response.message)
            return response

        response.success = self._send_pick_goal()
        response.message = '' if response.success else 'robot_arm_node 응답 없음'
        return response

    def emergency_stop_callback(self, msg):
        if msg.data and self._goal_handle is not None:
            self.get_logger().warn('긴급정지 - 진행중인 작업 취소 요청')
            self._goal_handle.cancel_goal_async()
            self._publish_status('EMERGENCY_STOP')

    def _send_pick_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self._publish_status('ERROR: robot_arm_node 응답 없음')
            return False

        self._busy = True
        self._publish_status('RUNNING')

        goal_msg = PickAndPlace.Goal()
        goal_msg.start = True

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._busy = False
            self._publish_status('ERROR: 작업 거부됨')
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback):
        fb = feedback.feedback
        if fb.event == PickAndPlace.Feedback.EVENT_DROPPED:
            self._publish_status(
                f'DROPPED: {fb.current_object} 이동중 물건이 떨어졌습니다. '
                f'컨베이어 벨트로 옮겨주세요.')
            return

        event_name = _EVENT_NAMES.get(fb.event, str(fb.event))
        self._publish_status(
            f'RUNNING: {event_name} object={fb.current_object} moved={fb.moved_count}')

    def _result_callback(self, future):
        result = future.result().result
        self._busy = False
        self._goal_handle = None
        if result.success:
            self._publish_status(f'DONE: total_moved={result.total_moved}')
        else:
            self._publish_status(f'STOPPED: total_moved={result.total_moved}')


def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from custom_interfaces.action import PickAndPlace
from custom_interfaces.srv import StartTask, UpdateInventory

_EVENT_NAMES = {
    PickAndPlace.Feedback.EVENT_PICKING: 'PICKING',
    PickAndPlace.Feedback.EVENT_GRABBED: 'GRABBED',
    PickAndPlace.Feedback.EVENT_PLACED: 'PLACED',
    PickAndPlace.Feedback.EVENT_RETRY: 'RETRY',
    PickAndPlace.Feedback.EVENT_DROPPED: 'DROPPED',
}
# YOLO class_id 기준 물품 목록 (db_node와 동일)
# class_id 0은 hands라서 재고 관리 대상에서 제외
ITEM_NAMES = {
    1: "TOY_TIMBER",
    2: "chocochip",
    3: "cotton_swab",
    4: "potato_chip",
    5: "pouch",
    6: "tooth_paste",
    7: "wet_wipes",
}

class CentralNode(Node):
    """작업 총괄 노드. ui_node의 버튼 입력을 받아 robot_arm_node에 피킹 액션을 요청한다."""

    def __init__(self):
        super().__init__('central_node')

        self._goal_handle = None
        self._busy = False                      # 작업 진행중이면 True

        self._action_client = ActionClient(self, PickAndPlace, '/pick')
        self.update_inventory_client = self.create_client(UpdateInventory, '/update_inventory')

        self._last_inventory_update_key = None

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
        self._last_inventory_update_key = None # 중복 방지 키 초기화
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

        # PLACED feedback을 받은 순간에만 DB 차감 요청
        if fb.event == PickAndPlace.Feedback.EVENT_PLACED:
            self._request_inventory_update(
                label_id=fb.current_label_id,
                item_name=fb.current_object,
                moved_count=int(fb.moved_count)
            )
    
    def _request_inventory_update(self, label_id, item_name, moved_count):
        if label_id not in ITEM_NAMES:
            self.get_logger().warn(f'DB 재고 차감 생략: 등록되지 않은 label_id={label_id}')
            return

        # 같은 PLACED feedback이 중복으로 들어와도 한 번만 차감
        update_key = (moved_count, label_id)

        if self._last_inventory_update_key == update_key:
            self.get_logger().warn(
                f'이미 차감한 PLACED feedback이므로 중복 차감 생략: {update_key}'
            )
            return

        if not self.update_inventory_client.service_is_ready():
            self.get_logger().warn('/update_inventory 서비스 준비 안 됨. db_node 확인 필요')
            return

        req = UpdateInventory.Request()
        req.item_name = str(item_name)
        req.item_class = int(label_id)

        future = self.update_inventory_client.call_async(req)
        future.add_done_callback(self._inventory_response_callback)

        self._last_inventory_update_key = update_key

    def _inventory_response_callback(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'DB 재고 차감 서비스 호출 실패: {exc}')
            return

        message = getattr(resp, 'message', f'remaining={resp.remaining}')

        if resp.success:
            self.get_logger().info(f'DB 재고 차감 성공: {message}')
        else:
            self.get_logger().warn(f'DB 재고 차감 실패: {message}')
        
    

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

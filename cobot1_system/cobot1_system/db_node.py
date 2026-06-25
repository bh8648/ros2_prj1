from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from custom_interfaces.srv import UpdateInventory


class DbNode(Node):

    def __init__(self):
        super().__init__('db_node')

        self.declare_parameter('initial_stock', 20)
        initial_stock = self.get_parameter('initial_stock').value

        # 처음 등장하는 item_name은 initial_stock 재고로 자동 초기화된다.
        self.inventory = defaultdict(lambda: initial_stock)
        self.log = []

        
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.create_service(UpdateInventory, '/update_inventory', self.update_inventory_callback)

        self.get_logger().info('db_node 시작됨 (재고·로그)')

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.get_logger().warn('긴급정지 신호 수신')

    def update_inventory_callback(self, request, response):
        item_name = request.item_name

        if self.inventory[item_name] <= 0:
            response.success = False
            response.remaining = 0
            self.get_logger().warn(f'재고 부족: {item_name}')
            return response

        self.inventory[item_name] -= 1
        response.success = True
        response.remaining = self.inventory[item_name]

        log_entry = f'[차감] {item_name} -1 (남은 수량: {response.remaining})'
        self.log.append(log_entry)
        self.get_logger().info(log_entry)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

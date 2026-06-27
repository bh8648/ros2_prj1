from pathlib import Path
import json
import sqlite3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String

from custom_interfaces.srv import UpdateInventory


# YOLO class_id 기준 물품 목록 (central_node와 동일)
# class_id 0은 hands라서 재고 관리 대상에서 제외
ITEM_NAMES= {
    1: "TOY_TIMBER",
    2: "chocochip",
    3: "cotton_swab",
    4: "potato_chip",
    5: "pouch",
    6: "tooth_paste",
    7: "wet_wipes",
}


class DbNode(Node):
    """
    재고 DB 관리 노드.

    역할:
        1. 실행 시 SQLite inventory 테이블 확인/생성
        2. 테이블이 비어 있으면 각 물품 수량을 initial_stock개로 초기화
        3. /update_inventory 서비스 요청을 받으면 해당 class_id 수량 -1
        4. 재고가 바뀌면 /inventory_state 토픽으로 UI에 최신 재고 전달

    통신:
        Subscribe:
            /emergency_stop    std_msgs/Bool

        Service Server:
            /update_inventory  custom_interfaces/srv/UpdateInventory

        Publish:
            /inventory_state   std_msgs/String
                JSON 문자열 예:
                [
                    {"class_id": 0, "item_name": "TOY_TIMBER", "quantity": 3},
                    {"class_id": 1, "item_name": "chocochip", "quantity": 2}
                ]
    """

    def __init__(self):
        super().__init__('db_node')

        self.declare_parameter('initial_stock', 3)
        self.declare_parameter(
            'db_path',
            str(Path.home() / '.ros' / 'cobot1_system' / 'inventory.db')
        )

        self.initial_stock = int(self.get_parameter('initial_stock').value)
        self.db_path = Path(str(self.get_parameter('db_path').value)).expanduser()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

        self.conn = sqlite3.connect(str(self.db_path))
        self.conn.row_factory = sqlite3.Row

        self.emergency_stopped = False

        self.init_database()

        # UI가 나중에 켜져도 마지막 재고 상태를 받을 수 있게 transient_local 사용
        inventory_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.inventory_pub = self.create_publisher(
            String,
            '/inventory_state',
            inventory_qos
        )

        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        self.create_service(
            UpdateInventory,
            '/update_inventory',
            self.update_inventory_callback
        )

        self.get_logger().info(f'db_node 시작됨: {self.db_path}')

        # 시작 직후 현재 재고 상태 발행
        self.publish_inventory_state()

    # ------------------------- DB 초기화 -------------------------
    def init_database(self):
        cur = self.conn.cursor()

        cur.execute("""
            CREATE TABLE IF NOT EXISTS inventory (
                class_id INTEGER PRIMARY KEY,
                item_name TEXT NOT NULL,
                quantity INTEGER NOT NULL
            )
        """)

        # 테이블에 없는 물품만 초기 수량으로 추가
        # 이미 존재하는 수량은 유지한다.
        for class_id, item_name in ITEM_NAMES.items():
            cur.execute(
                "SELECT class_id FROM inventory WHERE class_id = ?",
                (class_id,)
            )
            row = cur.fetchone()

            if row is None:
                cur.execute(
                    """
                    INSERT INTO inventory (class_id, item_name, quantity)
                    VALUES (?, ?, ?)
                    """,
                    (class_id, item_name, self.initial_stock)
                )

        self.conn.commit()

    # ------------------------- ROS 콜백 -------------------------
    def emergency_stop_callback(self, msg):
        self.emergency_stopped = bool(msg.data)

        if self.emergency_stopped:
            self.get_logger().warn('긴급정지 신호 수신')
        else:
            self.get_logger().info('긴급정지 해제 신호 수신')

    def update_inventory_callback(self, request, response):
        """
        central_node가 PLACED 성공 시 호출하는 서비스.

        추천 호출 예:
            ros2 service call /update_inventory custom_interfaces/srv/UpdateInventory \
            "{item_name: 'chocochip', item_class: 1}"

        재고 차감 기준:
            - request.item_class
        """

        item_name = str(getattr(request, 'item_name', '')).strip()

        item_class = int(request.item_class)

        if item_class is None or item_class not in ITEM_NAMES:
            message = f'등록되지 않은 물품: item_name={item_name}, item_class={item_class}'
            self.set_response(response, success=False, remaining=0, message=message)
            self.get_logger().warn(message)
            return response

        db_item_name = ITEM_NAMES[item_class]

        cur = self.conn.cursor()
        cur.execute(
            """
            SELECT quantity
            FROM inventory
            WHERE class_id = ?
            """,
            (item_class,)
        )
        row = cur.fetchone()

        if row is None:
            message = f'DB에 없는 class_id: {item_class}'
            self.set_response(response, success=False, remaining=0, message=message)
            self.get_logger().warn(message)
            return response

        current_qty = int(row['quantity'])

        if current_qty <= 0:
            message = f'재고 부족: {db_item_name}'
            self.set_response(response, success=False, remaining=0, message=message)
            self.get_logger().warn(message)
            self.publish_inventory_state()
            return response

        new_qty = current_qty - 1

        cur.execute(
            """
            UPDATE inventory
            SET quantity = ?
            WHERE class_id = ?
            """,
            (new_qty, item_class)
        )
        self.conn.commit()

        message = f'{db_item_name} -1 차감 완료, 남은 수량: {new_qty}'
        self.set_response(response, success=True, remaining=new_qty, message=message)

        self.get_logger().info(message)

        # 수량 변경 후 UI에 최신 재고 상태 발행
        self.publish_inventory_state()

        return response

    # ------------------------- 유틸 함수 -------------------------
    def find_class_id_by_name(self, item_name):
        for class_id, name in ITEM_NAMES.items():
            if name == item_name:
                return class_id
        return None

    def set_response(self, response, success, remaining, message=''):
        response.success = bool(success)
        response.remaining = int(remaining)

        # UpdateInventory.srv에 string message를 추가한 경우에만 사용
        # message 필드가 없는 기존 인터페이스여도 에러 나지 않게 처리
        if hasattr(response, 'message'):
            response.message = str(message)

    def publish_inventory_state(self):
        cur = self.conn.cursor()
        cur.execute("""
            SELECT class_id, item_name, quantity
            FROM inventory
            ORDER BY class_id
        """)
        rows = cur.fetchall()

        inventory_list = []
        for row in rows:
            inventory_list.append({
                'class_id': int(row['class_id']),
                'item_name': str(row['item_name']),
                'quantity': int(row['quantity']),
            })

        msg = String()
        msg.data = json.dumps(inventory_list, ensure_ascii=False)

        self.inventory_pub.publish(msg)
        self.get_logger().info(f'/inventory_state 발행: {msg.data}')

    def destroy_node(self):
        if hasattr(self, 'conn'):
            self.conn.close()
        super().destroy_node()


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

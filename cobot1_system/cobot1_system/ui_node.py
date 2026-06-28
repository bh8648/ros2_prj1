import os
import sys

import cv2
import numpy as np
import rclpy
import json
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from custom_interfaces.srv import StartTask

# opencv-python(pip)이 import 시 자기 Qt 플러그인 경로를 QT_QPA_PLATFORM_PLUGIN_PATH에
# 심어놓아 PyQt5의 xcb 플러그인 로딩과 충돌한다("Could not load platform plugin xcb").
# cv2/cv_bridge import 이후 이 경로를 지워 PyQt5가 자체 플러그인을 쓰게 한다.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap


class UiRosNode(Node):
    """central_node 계약에 맞춘 UI용 ROS 노드.

    - 작업 시작: /start_task (StartTask 서비스) 호출
    - 상태 수신: /central_status (String) 구독
    - 비상정지: /emergency_stop (Bool) 발행
    - 카메라 영상: /image_raw (Image) 구독
    """

    def __init__(self):
        super().__init__('ui_node')
        self.latest_frame = None
        self.latest_inventory = None

        self.start_client = self.create_client(StartTask, '/start_task') # 와 통신
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10) # 와 통신
        self.create_subscription(String, '/central_status', self.status_callback, 10) # central_node와 통신
        inventory_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, '/inventory_status', self.inventory_callback, inventory_qos)

        self.bridge = CvBridge()

        """
        '/yolo/position_image' : 인식된 장면 재생
        '/image_raw' : 순수 카메라 장면 재생
        """
        self.create_subscription(Image, '/yolo/position_image', self.image_callback, 10)

        self.latest_status = 'IDLE'

    def start_task(self):
        """작업 시작 서비스 호출. 서비스 미준비면 None, 아니면 future 반환."""
        if not self.start_client.service_is_ready():
            self.get_logger().warn('/start_task 서비스 준비 안 됨 (central_node 확인)')
            return None
        return self.start_client.call_async(StartTask.Request())

    def publish_estop(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.estop_pub.publish(msg)

    def status_callback(self, msg):
        self.latest_status = msg.data
    
    def inventory_callback(self, msg):
        try:
            self.latest_inventory = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"재고 상태 파싱 실패: {e}")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'이미지 변환 실패: {e}')


class BasketHMI(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self._last_status = None # central_node로부터 받아올 스테이터스
        self._last_inventory_text = None # db노드로부터 받아올 인벤토리 정보

        self.setWindowTitle("장바구니 물체 분류 HMI")
        self.resize(1300, 750)

        self.init_ui()

        # ROS 콜백 처리 + 화면 갱신 타이머
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin)
        self.ros_timer.start(20)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_status_from_ros)
        self.ui_timer.start(100)

        self.inventory_timer = QTimer()
        self.inventory_timer.timeout.connect(self.update_inventory_table)
        self.inventory_timer.start(100)

        self.cam_timer = QTimer()
        self.cam_timer.timeout.connect(self.update_camera)
        self.cam_timer.start(50)

    def init_ui(self):
        self.setStyleSheet("background-color: #000000;")

        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        title = QLabel("장바구니 물체 분류 시스템")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color:white; font-size:34px; font-weight:bold;")

        self.message = QLabel("장바구니를 올려주세요")
        self.message.setAlignment(Qt.AlignCenter)
        self.message.setStyleSheet("""
            background:#0B4F8A; color:white; font-size:38px; font-weight:bold;
            border-radius:20px; padding:60px;
        """)

        self.status = QLabel("상태 : 대기")
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet("color:white; font-size:24px; padding:15px;")

        # 버튼 디자인/레이아웃: ui_node2 기준 (작업시작 / 장바구니 치움 / 비상정지)
        self.btn_on = QPushButton("작업 시작")
        self.btn_on.setStyleSheet("""
            QPushButton{ background:#2E7D32; color:white; font-size:30px;
                font-weight:bold; padding:28px; border-radius:15px; }
            QPushButton:hover{ background:#43A047; }
        """)

        self.btn_clear = QPushButton("장바구니 치움")
        self.btn_clear.setStyleSheet("""
            QPushButton{ background:#1565C0; color:white; font-size:28px;
                font-weight:bold; padding:25px; border-radius:15px; }
            QPushButton:hover{ background:#1976D2; }
        """)

        self.btn_estop = QPushButton("비상 정지")
        self.btn_estop.setStyleSheet("""
            QPushButton{ background:#B71C1C; color:white; font-size:32px;
                font-weight:bold; padding:30px; border-radius:15px; }
            QPushButton:hover{ background:#D32F2F; }
        """)

        self.btn_on.clicked.connect(self.on_start)
        self.btn_clear.clicked.connect(self.on_clear)
        self.btn_estop.clicked.connect(self.on_estop)

        left_layout.addWidget(title)
        left_layout.addSpacing(20)
        left_layout.addWidget(self.message)
        left_layout.addWidget(self.status)
        left_layout.addSpacing(20)
        left_layout.addWidget(self.btn_on)
        left_layout.addWidget(self.btn_clear)
        left_layout.addStretch()
        left_layout.addWidget(self.btn_estop)   # 비상정지는 맨 아래

        # 카메라 창: ui_node2 기준 크기. 실제 영상은 update_camera에서 채운다.
        camera_title = QLabel("카메라 영상")
        camera_title.setAlignment(Qt.AlignCenter)
        camera_title.setStyleSheet("color:white; font-size:30px; font-weight:bold;")

        self.camera = QLabel("Camera")
        self.camera.setAlignment(Qt.AlignCenter)
        self.camera.setMinimumSize(560, 330)
        self.camera.setStyleSheet("""
            background:#333333; color:#DDDDDD; font-size:28px;
            border:2px solid gray; border-radius:15px;
        """)

        # 물건 목록/수량 표 (db 창). 화면엔 두되 실제 DB 연동은 아직 안 함 →
        # 기본 항목만 정적으로 표시. (TODO: /item_list 등으로 연동 시 update_item_table 추가)
        table_title = QLabel("물건 목록 / 수량")
        table_title.setStyleSheet("color:white; font-size:22px; font-weight:bold;")

        self.item_table = QTableWidget()
        self.item_table.setColumnCount(2)
        self.item_table.setHorizontalHeaderLabels(["물건 이름", "수량"])
        self.item_table.horizontalHeader().setStretchLastSection(True)
        self.item_table.setStyleSheet("""
            QTableWidget{ background:#111111; color:white; font-size:18px;
                gridline-color:#555555; border:1px solid #555555; }
            QHeaderView::section{ background:#222222; color:white; font-size:18px;
                font-weight:bold; border:1px solid #555555; padding:6px; }
        """)
        self.set_default_items()

        log_title = QLabel("작업 로그")
        log_title.setStyleSheet("color:white; font-size:22px; font-weight:bold;")

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet("background:#111111; color:white; font-size:18px; border:1px solid #555555;")
        self.log.append("[대기] 장바구니를 올려주세요")

        # 우측: 카메라 + 물건목록표 + 로그
        right_layout.addWidget(camera_title)
        right_layout.addWidget(self.camera)
        right_layout.addSpacing(10)
        right_layout.addWidget(table_title)
        right_layout.addWidget(self.item_table)
        right_layout.addSpacing(10)
        right_layout.addWidget(log_title)
        right_layout.addWidget(self.log)

        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 1)
        self.setLayout(main_layout)

    def set_default_items(self):
        "DB 상태를 받기 전까지 임시로 보여줄 기본 물건 목록."
        default_items = [
            ("초코칩", "99"),
            ("면봉", "99"),
            ("포테토칩", "99"),
            ("파우치", "99"),
            ("치약", "99"),
            ("물티슈", "99"),
            ("젠가", "99"),
        ]
        self.item_table.setRowCount(len(default_items))
        for row, (name, qty) in enumerate(default_items):
            self.item_table.setItem(row, 0, QTableWidgetItem(name))
            self.item_table.setItem(row, 1, QTableWidgetItem(qty))

    # ------------------------- 버튼 핸들러 -------------------------
    def on_start(self):
        self.log.append("[사용자] 작업 시작 버튼 클릭")
        self.ros_node.publish_estop(False)
        future = self.ros_node.start_task()
        if future is None:
            self.log.append("[오류] /start_task 서비스 준비 안 됨 (central_node 확인)")
            return
        future.add_done_callback(self._on_start_response)
        

    def _on_start_response(self, future):
        try:
            # ros_spin이 메인 스레드에서 돌므로 이 콜백도 메인 스레드 → Qt 위젯 접근 안전
            resp = future.result()
        except Exception as e:
            self.log.append(f"[오류] 작업 시작 서비스 호출 실패: {e}")
            return

        if resp.success:
            self.log.append("[ROS2] 작업 시작됨")
        else:
            self.log.append(f"[ROS2] 시작 실패: {resp.message}")

            

    def on_clear(self):
        self.message.setText("장바구니를 올려주세요")
        self.status.setText("상태 : 대기")
        self.log.append("[사용자] 장바구니 치움")

    def on_estop(self):
        self.ros_node.publish_estop(True)
        self.log.append("[사용자] 비상 정지 → /emergency_stop=true 발행")

    # ------------------------- 상태/영상 갱신 -------------------------
    def update_status_from_ros(self):
        s = self.ros_node.latest_status
        if s != self._last_status:
            self.log.append(f"[상태] {s}")
            self._last_status = s

        if s == 'IDLE':
            self.message.setText("장바구니를 올려주세요")
            self.status.setText("상태 : 대기")
        elif s.startswith('RUNNING'):
            if 'GRABBED' in s:
                self.message.setText("물체를 잡았습니다")
            elif 'PLACED' in s:
                self.message.setText("컨베이어로 옮기는 중")
            elif 'RETRY' in s:
                self.message.setText("잡기 재시도 중")
            else:
                self.message.setText("물체 인식/집는 중")
            self.status.setText(f"상태 : {s}")
        elif s.startswith('DROPPED'):
            self.message.setText("물건이 떨어졌습니다!\n컨베이어로 옮겨주세요")
            self.status.setText("상태 : 낙하")
        elif s.startswith('DONE'):
            self.message.setText("작업 완료 — 장바구니를 치워주세요")
            self.status.setText(f"상태 : {s}")
        elif s.startswith('EMERGENCY_STOP'):
            self.message.setText("비상 정지!")
            self.status.setText("상태 : 비상정지")
        elif s.startswith('STOPPED'):
            self.message.setText("작업 중단됨")
            self.status.setText(f"상태 : {s}")
        elif s.startswith('ERROR'):
            self.message.setText("오류 발생")
            self.status.setText(f"상태 : {s}")

    def update_inventory_table(self):
        inventory = self.ros_node.latest_inventory

        if not inventory:
            return

        inventory_text = str(inventory)
        if inventory_text == self._last_inventory_text:
            return
        self._last_inventory_text = inventory_text

        name_map = {
            "TOY_TIMBER": "젠가",
            "chocochip": "초코칩",
            "cotton_swab": "면봉",
            "potato_chip": "감자칩",
            "pouch": "파우치",
            "tooth_paste": "치약",
            "wet_wipes": "물티슈",
        }

        self.item_table.setRowCount(len(inventory))

        for row, item in enumerate(inventory):
            item_name = str(item.get("item_name", "unknown"))
            quantity = str(item.get("quantity", 0))

            display_name = name_map.get(item_name, item_name)

            self.item_table.setItem(row, 0, QTableWidgetItem(display_name))
            self.item_table.setItem(row, 1, QTableWidgetItem(quantity))

    def update_camera(self):
        frame = self.ros_node.latest_frame
        if frame is None:
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        h, w, ch = rgb.shape
        img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(img).scaled(
            self.camera.width(), self.camera.height(), Qt.KeepAspectRatio)
        self.camera.setPixmap(pix)

    def ros_spin(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def closeEvent(self, event):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    ros_node = UiRosNode()

    app = QApplication(sys.argv)
    window = BasketHMI(ros_node)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

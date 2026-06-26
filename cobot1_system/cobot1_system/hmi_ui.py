import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt


class BasketHMI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("장바구니 분류 HMI")
        self.resize(1100, 650)

        self.init_ui()

    def init_ui(self):
        main = QHBoxLayout()

        left = QVBoxLayout()
        right = QVBoxLayout()

        title = QLabel("장바구니 물체 분류 시스템")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 32px; font-weight: bold; color: white;")

        self.message = QLabel("장바구니를 올려주세요")
        self.message.setAlignment(Qt.AlignCenter)
        self.message.setStyleSheet("""
            font-size: 36px;
            font-weight: bold;
            color: white;
            background-color: #0b4f8a;
            border-radius: 20px;
            padding: 60px;
        """)

        self.status = QLabel("상태: 대기 중")
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet("font-size: 24px; color: white;")

        btn_basket_on = QPushButton("장바구니 올림")
        btn_basket_on.setStyleSheet("""
            font-size: 28px;
            font-weight: bold;
            color: white;
            background-color: #2d7d12;
            border-radius: 15px;
            padding: 25px;
        """)
        btn_basket_on.clicked.connect(self.basket_on)

        btn_basket_clear = QPushButton("장바구니 치움")
        btn_basket_clear.setStyleSheet("""
            font-size: 28px;
            font-weight: bold;
            color: white;
            background-color: #2d7d12;
            border-radius: 15px;
            padding: 25px;
        """)
        btn_basket_clear.clicked.connect(self.basket_clear)

        btn_emergency = QPushButton("긴급 정지")
        btn_emergency.setStyleSheet("""
            font-size: 32px;
            font-weight: bold;
            color: white;
            background-color: #9b1c1c;
            border-radius: 15px;
            padding: 30px;
        """)
        btn_emergency.clicked.connect(self.emergency_stop)

        left.addWidget(title)
        left.addWidget(self.message)
        left.addWidget(self.status)
        left.addWidget(btn_basket_on)
        left.addWidget(btn_basket_clear)
        left.addStretch()
        left.addWidget(btn_emergency)

        camera_title = QLabel("카메라 영상")
        camera_title.setAlignment(Qt.AlignCenter)
        camera_title.setStyleSheet("font-size: 28px; font-weight: bold; color: white;")

        self.camera = QLabel("카메라 화면")
        self.camera.setAlignment(Qt.AlignCenter)
        self.camera.setStyleSheet("""
            font-size: 30px;
            color: #dddddd;
            background-color: #333333;
            border: 2px solid #aaaaaa;
            border-radius: 15px;
        """)
        self.camera.setMinimumSize(500, 380)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet("""
            font-size: 18px;
            background-color: #111111;
            color: white;
            border: 1px solid #555555;
        """)
        self.log.append("[대기] 장바구니를 올려주세요")

        right.addWidget(camera_title)
        right.addWidget(self.camera)
        right.addWidget(QLabel("작업 로그"))
        right.addWidget(self.log)

        main.addLayout(left, 1)
        main.addLayout(right, 1)

        self.setLayout(main)
        self.setStyleSheet("background-color: #000000;")

    def basket_on(self):
        self.message.setText("YOLO Segment 물체 인식 중")
        self.status.setText("상태: 물체 인식")
        self.log.append("[입력] 장바구니 올림 버튼 클릭")
        self.log.append("[처리] YOLO Segment 물체 인식 중")

    def basket_clear(self):
        self.message.setText("장바구니를 올려주세요")
        self.status.setText("상태: 처음으로 복귀")
        self.log.append("[입력] 장바구니 치움 버튼 클릭")
        self.log.append("[대기] 장바구니를 올려주세요")

    def emergency_stop(self):
        self.message.setText("긴급 정지")
        self.status.setText("상태: 모든 동작 정지")
        self.message.setStyleSheet("""
            font-size: 40px;
            font-weight: bold;
            color: white;
            background-color: #9b1c1c;
            border-radius: 20px;
            padding: 60px;
        """)
        self.log.append("[긴급 정지] 모든 동작 정지")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BasketHMI()
    window.show()
    sys.exit(app.exec_())

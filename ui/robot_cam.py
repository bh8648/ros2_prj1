import os
import sys
import cv2

# Qt 환경 설정 (PyQt import 전에 위치)
os.environ["QT_QPA_PLATFORM"] = "xcb"

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # UI 로드
        uic.loadUi("robot_cam.ui", self)

        # 카메라 객체
        self.cap = None

        # 타이머 (영상 루프)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        # 버튼 연결
        self.btnStartCam.clicked.connect(self.start_camera)
        self.btnStopCam.clicked.connect(self.stop_camera)

    def start_camera(self):
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("카메라 열기 실패")
            self.cap = None
            return

        self.timer.start(30)

    def stop_camera(self):
        self.timer.stop()

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.videoLabel.clear()
        self.videoLabel.setText("Camera Off")

    def update_frame(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape
        bytes_per_line = ch * w

        qimg = QImage(frame.copy().data, w, h, bytes_per_line, QImage.Format_RGB888)

        self.videoLabel.setPixmap(QPixmap.fromImage(qimg))


def main():
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

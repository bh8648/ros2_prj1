import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ultralytics import YOLO

from custom_interfaces.srv import DetectObject


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 2)
        self.declare_parameter('model_path', 'yolov26n.pt')
        self.declare_parameter('confidence_threshold', 0.6)

        camera_index = self.get_parameter('camera_index').value
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'카메라(index={camera_index})를 열 수 없습니다.')                              # ui에 상태창 띄우는 함수에 연결.

        self.model = YOLO(model_path)
        self.latest_frame = None

        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.create_service(DetectObject, '/detect_object', self.detect_object_callback)
        self.create_timer(0.1, self.capture_timer_callback)

        self.get_logger().info('camera_node 시작됨 (YOLO 인식)')

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.get_logger().warn('긴급정지 신호 수신')

    def capture_timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.latest_frame = frame
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(msg)

    def detect_object_callback(self, request, response):
        response.found = False
        response.center_x = 0.0
        response.center_y = 0.0
        response.angle = 0.0
        response.label = ''

        if not request.capture or self.latest_frame is None:
            return response

        frame = self.latest_frame
        results = self.model(frame, verbose=False)

        best_box = None
        best_conf = 0.0
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < self.confidence_threshold or conf <= best_conf:
                    continue
                best_conf = conf
                best_box = box

        if best_box is None:
            return response

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        cls_id = int(best_box.cls[0])
        label = self.model.names[cls_id]

        response.found = True
        response.center_x = (x1 + x2) / 2.0
        response.center_y = (y1 + y2) / 2.0
        response.angle = self._estimate_angle(frame, x1, y1, x2, y2)
        response.label = str(label)

        self.get_logger().info(
            f'detect_object: label={label}, '
            f'center=({response.center_x:.1f},{response.center_y:.1f}), '
            f'angle={response.angle:.1f}'
        )
        return response

    def _estimate_angle(self, frame, x1, y1, x2, y2):
        """bbox 내부 최대 컨투어의 minAreaRect 각도로 물건 기울기를 근사한다."""
        h, w = frame.shape[:2]
        x1, y1 = max(int(x1), 0), max(int(y1), 0)
        x2, y2 = min(int(x2), w), min(int(y2), h)
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return 0.0

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 10:
            return 0.0

        return float(cv2.minAreaRect(largest)[-1])

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

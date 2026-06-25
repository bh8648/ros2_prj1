import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ultralytics import YOLO

from custom_interfaces.srv import DetectObject


class CameraNode(Node):
    """
    camera_node.py 기준 통신 이름을 유지한 YOLO segmentation 서비스 노드.

    Publish:
      - /image_raw              sensor_msgs/Image        원본 카메라 프레임
      - /yolo/position_image    sensor_msgs/Image        YOLO 결과 시각화 이미지

    Subscribe:
      - /emergency_stop         std_msgs/Bool            긴급정지 신호

    Service:
      - /detect_object          custom_interfaces/srv/DetectObject

    DetectObject.srv:
      bool capture
      ---
      bool found
      float64 center_x
      float64 center_y
      float64 angle
      string label
    """

    def __init__(self):
        super().__init__('camera_node')

        # --------------------------------------------------
        # camera_node.py 기준 파라미터
        # --------------------------------------------------
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('model_path', 'trained_yolo26_seg_best.pt')
        self.declare_parameter('confidence_threshold', 0.75)

        # 통신 이름도 파라미터로 열어두되, 기본값은 camera_node.py 기준으로 고정
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('detect_service_name', '/detect_object')
        self.declare_parameter('debug_image_topic', '/yolo/position_image')
        self.declare_parameter('publish_debug_image', True)

        camera_index = int(self.get_parameter('camera_index').value)
        self.get_logger().info(f"카메라 인덱스{camera_index}")
        model_path_param = str(self.get_parameter('model_path').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.emergency_stop_topic = str(
            self.get_parameter('emergency_stop_topic').value
        )
        self.detect_service_name = str(
            self.get_parameter('detect_service_name').value
        )
        self.debug_image_topic = str(
            self.get_parameter('debug_image_topic').value
        )
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value
        )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.emergency_stop_requested = False

        # --------------------------------------------------
        # Camera open
        # --------------------------------------------------
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'카메라(index={camera_index})를 열 수 없습니다.'
            )

        # --------------------------------------------------
        # YOLO model load
        # --------------------------------------------------
        self.model_path = self._resolve_model_path(model_path_param)
        self.get_logger().info(f'YOLO model path: {self.model_path}')
        self.model = YOLO(self.model_path)

        # --------------------------------------------------
        # ROS communication - camera_node.py 기준 이름
        # --------------------------------------------------
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)

        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                10
            )

        self.create_subscription(
            Bool,
            self.emergency_stop_topic,
            self.emergency_stop_callback,
            10
        )

        self.create_service(
            DetectObject,
            self.detect_service_name,
            self.detect_object_callback
        )

        self.create_timer(0.1, self.capture_timer_callback)

        self.get_logger().info(
            'camera_node 시작됨 '
            f'image_pub={self.image_topic}, '
            f'emergency_sub={self.emergency_stop_topic}, '
            f'service={self.detect_service_name}, '
            f'debug_image={self.debug_image_topic}'
        )

    def _resolve_model_path(self, model_path_param):
        """
        model_path가 절대경로면 그대로 사용.
        파일명만 들어오면 cobot1_system/share/models 아래를 우선 확인.
        """
        if os.path.isabs(model_path_param) and os.path.exists(model_path_param):
            return model_path_param

        if os.path.exists(model_path_param):
            return model_path_param

        try:
            package_model_path = os.path.join(
                get_package_share_directory('cobot1_system'),
                'models',
                model_path_param
            )
            if os.path.exists(package_model_path):
                return package_model_path
        except Exception as exc:
            self.get_logger().warn(
                f'yolo_pub share directory 확인 실패: {exc}'
            )

        # 마지막 fallback: YOLO가 직접 해석하도록 원래 문자열 반환
        return model_path_param

    # --------------------------------------------------
    # Topic callbacks
    # --------------------------------------------------
    def emergency_stop_callback(self, msg):
        self.emergency_stop_requested = bool(msg.data)
        if self.emergency_stop_requested:
            self.get_logger().warn('긴급정지 신호 수신: detect_object 응답을 found=False로 반환합니다.')
        else:
            self.get_logger().info('긴급정지 해제 신호 수신')

    def capture_timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('카메라 프레임을 읽지 못했습니다.')
            return

        self.latest_frame = frame

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.frame_id = 'camera_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)

    # --------------------------------------------------
    # DetectObject service
    # --------------------------------------------------
    def detect_object_callback(self, request, response):
        response.found = False
        response.center_x = 0.0
        response.center_y = 0.0
        response.angle = 0.0
        response.label = ''

        if not request.capture:
            self.get_logger().info('detect_object 요청 수신: capture=False')
            return response

        if self.emergency_stop_requested:
            self.get_logger().warn('긴급정지 상태이므로 detect_object를 수행하지 않습니다.')
            return response

        if self.latest_frame is None:
            self.get_logger().warn('아직 수신된 카메라 프레임이 없습니다.')
            return response

        frame = self.latest_frame.copy()
        detection, vis_frame = self._detect_best_object(frame)

        if self.publish_debug_image and vis_frame is not None:
            self._publish_debug_image(vis_frame)

        if detection is None:
            self.get_logger().info('detect_object: 검출된 물체 없음')
            return response

        response.found = True
        response.center_x = float(detection['center_x'])
        response.center_y = float(detection['center_y'])
        response.angle = float(detection['angle'])
        response.label = str(detection['label'])

        self.get_logger().info(
            f"detect_object: label={response.label}, "
            f"center=({response.center_x:.1f}, {response.center_y:.1f}), "
            f"angle={response.angle:.1f}, "
            f"conf={detection['confidence']:.3f}"
        )

        return response

    def _detect_best_object(self, frame):
        """
        !!!!! 고쳐야 함
        카메라 노드에서 어떤 물건을 대상으로 그리핑을 할지 우선순위 선별하고 대상이 된 물건의 정보를 여기서 줌.

        """
        
        """
        현재 프레임에서 confidence가 가장 높은 객체 1개를 DetectObject 응답 형태로 변환.
        segmentation mask가 있으면 mask 중심/PCA 각도를 우선 사용하고,
        mask가 없으면 bbox 중심/minAreaRect 각도로 fallback.
        """
        h, w = frame.shape[:2]
        vis_frame = frame.copy()

        results = self.model(
            frame,
            conf=self.confidence_threshold,
            verbose=False,
            retina_masks=True
        )

        best_detection = None

        for result in results:
            boxes = result.boxes
            masks = result.masks

            if boxes is None or len(boxes) == 0:
                continue

            for i, box in enumerate(boxes):
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                bbox_center_x = (x1 + x2) / 2.0
                bbox_center_y = (y1 + y2) / 2.0

                cls_id = int(box.cls[0])
                label = str(self.model.names[cls_id])

                center_x = bbox_center_x
                center_y = bbox_center_y
                angle = None
                binary_mask = None

                if masks is not None and masks.data is not None and i < len(masks.data):
                    raw_mask = masks.data[i].cpu().numpy()

                    if raw_mask.shape != (h, w):
                        raw_mask = cv2.resize(
                            raw_mask,
                            (w, h),
                            interpolation=cv2.INTER_NEAREST
                        )

                    binary_mask = (raw_mask > 0.5).astype(np.uint8)
                    centroid = self._get_mask_centroid(binary_mask)
                    if centroid is not None:
                        center_x, center_y = centroid

                    angle = self._get_mask_angle_pca(binary_mask)

                if angle is None:
                    angle = self._estimate_angle_from_bbox(frame, x1, y1, x2, y2)

                detection = {
                    'label': label,
                    'confidence': confidence,
                    'center_x': float(center_x),
                    'center_y': float(center_y),
                    'angle': float(angle),
                    'bbox': (x1, y1, x2, y2),
                    'mask': binary_mask,
                }

                if (
                    best_detection is None
                    or detection['confidence'] > best_detection['confidence']
                ):
                    best_detection = detection

        if best_detection is not None:
            vis_frame = self._draw_detection(vis_frame, best_detection)

        return best_detection, vis_frame

    # --------------------------------------------------
    # Geometry helpers
    # --------------------------------------------------
    def _get_mask_centroid(self, binary_mask):
        mask_uint8 = binary_mask.astype(np.uint8)
        moment = cv2.moments(mask_uint8)

        if moment['m00'] == 0:
            return None

        cx = moment['m10'] / moment['m00']
        cy = moment['m01'] / moment['m00']
        return float(cx), float(cy)

    def _get_mask_angle_pca(self, binary_mask):
        ys, xs = np.where(binary_mask > 0)

        if len(xs) < 10:
            return None

        points = np.column_stack((xs, ys)).astype(np.float32)
        _, eigenvectors = cv2.PCACompute(points, mean=None)

        vx, vy = eigenvectors[0]
        angle_deg = math.degrees(math.atan2(vy, vx))

        # PCA 장축은 180도 차이가 같은 축이므로 -90~90으로 정규화
        if angle_deg > 90:
            angle_deg -= 180
        elif angle_deg < -90:
            angle_deg += 180

        return float(angle_deg)

    def _estimate_angle_from_bbox(self, frame, x1, y1, x2, y2):
        """segmentation mask가 없을 때 bbox 내부 컨투어로 기울기 fallback."""
        h, w = frame.shape[:2]
        x1 = max(int(x1), 0)
        y1 = max(int(y1), 0)
        x2 = min(int(x2), w)
        y2 = min(int(y2), h)

        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return 0.0

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return 0.0

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 10:
            return 0.0

        return float(cv2.minAreaRect(largest)[-1])

    # --------------------------------------------------
    # Visualization
    # --------------------------------------------------
    def _draw_detection(self, frame, detection):
        x1, y1, x2, y2 = detection['bbox']
        cx = detection['center_x']
        cy = detection['center_y']
        angle = detection['angle']
        label = detection['label']
        confidence = detection['confidence']
        binary_mask = detection['mask']

        vis = frame.copy()

        if binary_mask is not None:
            overlay = vis.copy()
            overlay[binary_mask > 0] = (0, 255, 0)
            vis = cv2.addWeighted(overlay, 0.3, vis, 0.7, 0)

        cv2.rectangle(
            vis,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            (255, 0, 0),
            2
        )

        cv2.circle(vis, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        cv2.putText(
            vis,
            f'{label} {confidence:.2f}',
            (int(x1), max(int(y1) - 25, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2
        )

        cv2.putText(
            vis,
            f'center=({cx:.1f},{cy:.1f})',
            (int(x1), max(int(y1) - 5, 40)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

        line_len = 60
        rad = math.radians(angle)
        x_end = int(cx + line_len * math.cos(rad))
        y_end = int(cy + line_len * math.sin(rad))
        cv2.line(vis, (int(cx), int(cy)), (x_end, y_end), (0, 255, 255), 2)

        cv2.putText(
            vis,
            f'{angle:.1f} deg',
            (int(cx) + 10, int(cy) + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2
        )

        return vis

    def _publish_debug_image(self, frame):
        if self.debug_image_pub is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.frame_id = 'camera_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub.publish(msg)

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

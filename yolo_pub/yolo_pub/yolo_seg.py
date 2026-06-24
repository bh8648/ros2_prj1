import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(String, 'detection/result', 10)
        self.publiser_image = self.create_publisher(Image, 'yolo/yolo_image', 10)

        self.bridge = CvBridge()
        
        package_share_dir = get_package_share_directory('yolo_pub')

        self.yolo_path_name = "trained_yolo26_seg_multiclass_20260623_145043_last.pt"

        self.yolo_path = str(
            Path(package_share_dir) / "models" / self.yolo_path_name
        )
        self.model = YOLO(self.yolo_path)


    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(frame)

        for r in results:
            boxes = r.boxes
            masks = r.masks

            if masks is not None and len(masks) > 0:
                H, W, _ = frame.shape
                # 모든 객체의 마스크를 합칠 도화지 생성
                combined_mask = np.zeros((H, W), dtype=np.float32)

                for mask in masks:
                    # 개별 마스크를 원본 frame 크기에 맞게 리사이즈하여 도화지에 병합
                    raw_mask = mask.data[0].cpu().numpy()
                    binary_mask = cv2.resize(raw_mask, (W, H))
                    combined_mask = np.maximum(combined_mask, binary_mask)
                
                
                # 객체가 존재하는 픽셀 인덱스 추출 (0.5 기준 이진화)
                idx = combined_mask > 0.5

                # 원본 frame 위에 직접 초록색 반투명 필터 적용 (투명도 30%)
                alpha = 0.3
                frame[idx] = (frame[idx] * (1 - alpha) + np.array([0, 255, 0]) * alpha).astype(np.uint8)
           
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publiser_image.publish(img_msg)


            for box in boxes:

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                confi = box.conf[0]
                width = x2 - x1
                height = y2 - y1

                cx = x1 + width / 2
                cy = y1 + height / 2

                text = f"{label}, center=({cx:.1f},{cy:.1f}), width={width:.1f}, height={height:.1f}"
                text2 = f"{label}\nconf={confi:.2f}"
                cv2.putText(frame, text2, (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (100,55,0),
                            2)


                msg = String()
                msg.data = text
                self.publisher.publish(msg)
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publiser_image.publish(img_msg)

def main(args=None):

    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
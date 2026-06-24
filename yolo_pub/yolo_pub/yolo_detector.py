import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

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
        self.model = YOLO("yolov8n.pt")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(frame)

        for r in results:
            boxes = r.boxes

            for box in boxes:

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                width = x2 - x1
                height = y2 - y1

                cx = x1 + width / 2
                cy = y1 + height / 2

                text = f"{label}, center=({cx:.1f},{cy:.1f}), width={width:.1f}, height={height:.1f}"

                # msg2 = Image()
                # msg2.data = results
                # self.publisher_image.publish(msg2)
                self.get_logger().log(results.shape)
                msg = String()
                msg.data = text
                self.publisher.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
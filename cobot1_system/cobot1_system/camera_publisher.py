import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)

        self.declare_parameter('camera_index', 2)
        camera_index = int(self.get_parameter('camera_index').value)
        self.get_logger().info(f"카메라 인덱스{camera_index}")

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'카메라(index={camera_index})를 열 수 없습니다.'
            )

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

        
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('카메라 프레임을 읽지 못했습니다.')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.frame_id = 'camera_frame'
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
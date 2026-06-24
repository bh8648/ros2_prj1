import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ResultSubscriber(Node):

    def __init__(self):
        super().__init__('result_subscriber')

        self.subscription = self.create_subscription(
            String,
            'detection/result',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print("Detection Result:", msg.data)

def main(args=None):

    rclpy.init(args=args)
    node = ResultSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
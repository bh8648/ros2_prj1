#!/usr/bin/env python3
"""
컨베이어 벨트 ROS2 구독자 노드
- /conveyor/run 토픽 수신 (std_msgs/Bool, True 일 때만 동작)
- 아두이노에 'G' 명령 전송 -> fire-and-forget (응답 대기 없음)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # std_srvs.srv -> std_msgs.msg로 변경
import serial
import time


class ConveyorNode(Node):
    def __init__(self):
        super().__init__("conveyor_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("timeout", 15.0)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        self.timeout = self.get_parameter("timeout").value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=self.timeout)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Arduino connected: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        self.sub = self.create_subscription(
            Bool, "/conveyor/run", self.run_callback, 10
        )
        self.get_logger().info("Conveyor Service Ready (/conveyor/run)")

    def run_callback(self, msg):
        # True 신호가 아니면 무시
        if not msg.data:
            return

        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial not connected - 컨베이어 구동 생략")
            return

        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"G")
            # 아두이노가 10초 구동 후 알아서 멈춤. 여기선 응답 대기 없이 바로 반환
            self.get_logger().info("Conveyor started (10s run, fire-and-forget)")
        except Exception as e:
            self.get_logger().error(f"Conveyor write error: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

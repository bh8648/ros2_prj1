#!/usr/bin/env python3
"""
컨베이어 벨트 ROS2 서비스 서버
- /conveyor/run 서비스 요청 수신 (std_srvs/Trigger)
- 아두이노에 'G' 명령 전송 -> 10초 구동 -> 'DONE' 응답 수신
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import time


class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 15.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=self.timeout)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Arduino connected: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.ser = None

        self.srv = self.create_service(Trigger, '/conveyor/run', self.run_callback)
        self.get_logger().info('Conveyor Service Ready (/conveyor/run)')

    def run_callback(self, request, response):
        if self.ser is None or not self.ser.is_open:
            response.success = False
            response.message = 'Serial not connected'
            return response

        try:
            self.ser.reset_input_buffer()
            self.ser.write(b'G')
            self.get_logger().info('Conveyor started (10s run)')

            line = self.ser.readline().decode().strip()

            if line == 'DONE':
                response.success = True
                response.message = 'Conveyor run completed'
                self.get_logger().info('Conveyor finished')
            else:
                response.success = False
                response.message = f'Unexpected response: {line}'
        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'

        return response

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


if __name__ == '__main__':
    main()
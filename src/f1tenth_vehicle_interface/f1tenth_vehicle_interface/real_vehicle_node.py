#!/usr/bin/env python3
"""
실차용 vehicle interface (Jetson Orin Nano + VESC).

/drive(AckermannDriveStamped)를 받아 f1tenth_system(VESC 드라이버)이 기대하는
토픽으로 변환/중계하는 노드. f1tenth_system(https://github.com/f1tenth/f1tenth_system)을
설치했다면 보통 vesc_ackermann 패키지가 /drive를 이미 직접 구독하므로,
이 노드는 안전 리밋(속도 clamp, 워치독) 정도만 담당하는 게 실전에서 안전합니다.

TODO(백종민):
  - f1tenth_system 설치 후 실제 사용 중인 토픽/파라미터 이름 확인해서 맞추기
  - 워치독: N초 이상 /drive가 안 들어오면 자동 정지
  - 저속 안전 테스트부터 시작 (max_speed 파라미터를 낮게)
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped


class RealVehicleNode(Node):
    def __init__(self):
        super().__init__('real_vehicle_node')
        self.declare_parameter('watchdog_timeout_sec', 0.5)
        self.declare_parameter('max_speed_safety_cap', 3.0)

        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self.max_speed_cap = self.get_parameter('max_speed_safety_cap').value

        self.sub = self.create_subscription(AckermannDriveStamped, '/drive', self._drive_cb, 10)
        # TODO: 실제 VESC 인터페이스가 구독하는 토픽으로 재발행 (예: /vesc/ackermann_cmd)
        self.pub = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd', 10)

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog_check)

        self.get_logger().warn(
            'real_vehicle_node: /vesc/ackermann_cmd 토픽 이름을 실제 f1tenth_system 설정과 확인하세요.'
        )

    def _drive_cb(self, msg: AckermannDriveStamped):
        self.last_cmd_time = self.get_clock().now()
        if abs(msg.drive.speed) > self.max_speed_cap:
            msg.drive.speed = self.max_speed_cap if msg.drive.speed > 0 else -self.max_speed_cap
            self.get_logger().warn('안전 리밋으로 speed clamp됨')
        self.pub.publish(msg)

    def _watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.pub.publish(stop_msg)


def main():
    rclpy.init()
    node = RealVehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

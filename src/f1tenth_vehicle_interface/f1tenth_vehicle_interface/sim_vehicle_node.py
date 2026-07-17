#!/usr/bin/env python3
"""
시뮬레이션용 vehicle interface.

이미 쓰고 있는 시뮬레이터(f1tenth_gym_ros 등)가 자체적으로 /drive를 구독한다면
이 노드는 사실 필요 없고, launch에서 그 시뮬레이터 노드를 대신 include하면 됩니다.
지금은 최소 동작 확인용으로, /drive를 받아서 콘솔에 echo만 하는 더미로 구성했습니다.
팀에서 쓰는 시뮬레이터가 정해지면 이 파일을 그 시뮬레이터 launch include로 교체하세요.
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SimVehicleNode(Node):
    def __init__(self):
        super().__init__('sim_vehicle_node')
        self.create_subscription(AckermannDriveStamped, '/drive', self._drive_cb, 10)
        self.get_logger().warn(
            '더미 sim_vehicle_node 실행 중. 실제 시뮬레이터(f1tenth_gym_ros 등) 연동으로 교체 필요.'
        )

    def _drive_cb(self, msg: AckermannDriveStamped):
        self.get_logger().info(
            f'[SIM] steer={msg.drive.steering_angle:.3f} speed={msg.drive.speed:.3f}',
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = SimVehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

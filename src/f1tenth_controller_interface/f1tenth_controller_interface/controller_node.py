#!/usr/bin/env python3
"""
MPPI 컨트롤러 조원(백종민/박성현)용 골격 노드.

*** 이 파일의 토픽 이름/타입은 팀 전체 계약이므로 임의로 바꾸지 마세요. ***
아래 3개를 구독하고 1개를 발행하는 구조만 지키면, 시뮬/실차 어디서든
f1tenth_bringup의 launch에 그대로 꽂힙니다.

구독:
  /scan          sensor_msgs/LaserScan          - 장애물 회피용
  /odom          nav_msgs/Odometry              - 현재 차량 상태(x,y,yaw,v)
  /global_path   nav_msgs/Path                  - f1tenth_waypoints가 발행하는 센터라인

발행:
  /drive         ackermann_msgs/AckermannDriveStamped

MPPI 내부 로직(샘플링, cost function, rollout 등)은 이 안에서 자유롭게 구현하면 됩니다.
Nav2 MPPI critic 구조(CostCritic, PathFollowCritic 등)를 참고해서
_compute_cost() 부분만 채워나가는 방식을 추천합니다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped


class MPPIControllerNode(Node):
    def __init__(self):
        super().__init__('mppi_controller')

        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('num_samples', 1024)
        self.declare_parameter('horizon_steps', 30)
        self.declare_parameter('dt', 0.05)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.num_samples = self.get_parameter('num_samples').value
        self.horizon = self.get_parameter('horizon_steps').value
        self.dt = self.get_parameter('dt').value

        self.latest_scan: LaserScan | None = None
        self.latest_odom: Odometry | None = None
        self.latest_path: Path | None = None

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Path, '/global_path', self._path_cb, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('MPPI controller skeleton 시작. TODO: _compute_cost() 구현 필요.')

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _path_cb(self, msg: Path):
        self.latest_path = msg

    def _control_loop(self):
        if self.latest_odom is None or self.latest_path is None:
            return  # 아직 데이터가 안 모임

        # ------------------------------------------------------------------
        # TODO(조원): 여기부터 실제 MPPI 구현
        #   1) 현재 상태(x, y, yaw, v) 추출
        #   2) num_samples개의 제어 시퀀스(steer, speed)를 노이즈 섞어 샘플링
        #   3) kinematic bicycle model로 horizon_steps만큼 rollout
        #   4) _compute_cost()로 각 rollout 평가 (path follow + collision + smoothness)
        #   5) cost 가중 평균으로 최적 제어 시퀀스 계산, 첫 스텝만 실행
        # 아래는 자리 표시용 더미 제어(정지)입니다. 반드시 교체하세요.
        # ------------------------------------------------------------------
        steer_cmd, speed_cmd = self._dummy_control()

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(np.clip(steer_cmd, -self.max_steer, self.max_steer))
        drive_msg.drive.speed = float(np.clip(speed_cmd, 0.0, self.max_speed))
        self.drive_pub.publish(drive_msg)

    def _dummy_control(self):
        """TODO: MPPI 구현 전까지의 안전한 placeholder (정지)."""
        return 0.0, 0.0

    def _compute_cost(self, rollout_states, rollout_controls):
        """
        TODO(조원): critic 구조 참고 구현.
          - PathFollowCost: rollout 지점과 /global_path 사이 거리
          - CollisionCost: /scan 기반 장애물까지 거리
          - SmoothnessCost: 조향각 변화율 페널티
        """
        raise NotImplementedError


def main():
    rclpy.init()
    node = MPPIControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

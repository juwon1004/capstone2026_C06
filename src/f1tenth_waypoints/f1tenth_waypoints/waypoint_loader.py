#!/usr/bin/env python3
"""
extract_centerline.py로 만든 CSV를 읽어 nav_msgs/Path(/global_path)로 계속 퍼블리시하는 노드.
컨트롤러(MPPI)는 이 /global_path 토픽 하나만 구독하면 됨 (race_stack의 planner 대응).
"""
import csv
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WaypointLoader(Node):
    def __init__(self):
        super().__init__('waypoint_loader')
        self.declare_parameter('csv_path', 'config/centerline.csv')
        self.declare_parameter('loop', True)
        self.declare_parameter('publish_rate_hz', 2.0)

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.path_msg = self._load_csv(csv_path)
        self.pub = self.create_publisher(Path, '/global_path', 1)
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'{len(self.path_msg.poses)}개 waypoint 로드 완료: {csv_path}')

    def _load_csv(self, csv_path: str) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        if not os.path.exists(csv_path):
            self.get_logger().error(f'CSV를 찾을 수 없음: {csv_path} (extract_centerline.py 먼저 실행)')
            return path
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        if self.loop and path.poses:
            path.poses.append(path.poses[0])
        return path

    def _publish(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = WaypointLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

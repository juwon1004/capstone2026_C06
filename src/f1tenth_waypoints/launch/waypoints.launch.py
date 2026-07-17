import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='lab_track')
    return LaunchDescription([
        map_name_arg,
        Node(
            package='f1tenth_waypoints',
            executable='waypoint_loader',
            name='waypoint_loader',
            output='screen',
            parameters=[{
                'csv_path': [ 'config/', LaunchConfiguration('map_name'), '_centerline.csv' ],
                'loop': True,
                'publish_rate_hz': 2.0,
            }],
        ),
    ])

"""
F1TENTH 캡스톤 통합 launch.

핵심 설계: sim:=true/false 인자 하나로 map->localization->waypoints->controller는
그대로 재사용하고, 맨 끝단(vehicle_interface)만 시뮬레이터 <-> 실차 노드로 스왑한다.

사용 예:
  ros2 launch f1tenth_bringup bringup.launch.py sim:=true  map_name:=lab_track
  ros2 launch f1tenth_bringup bringup.launch.py sim:=false map_name:=lab_track
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sim_arg = DeclareLaunchArgument('sim', default_value='true',
                                     description='true=시뮬레이션, false=실차')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='lab_track',
                                          description='f1tenth_maps/maps/<map_name>.yaml')

    sim = LaunchConfiguration('sim')
    map_name = LaunchConfiguration('map_name')

    maps_dir = get_package_share_directory('f1tenth_maps') if _pkg_exists('f1tenth_maps') else \
        os.path.join(os.path.dirname(__file__), '..', '..', 'f1tenth_maps', 'maps')

    # 1) 맵 로드 (시뮬/실차 공통)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': PathJoinSubstitution([maps_dir, map_name, '.yaml'])
        }],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}],
    )

    # 2) localization: slam_toolbox를 "localization" 모드로 (map->odom 발행, 시뮬/실차 공통)
    localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[{'map_file_name': PathJoinSubstitution([maps_dir, map_name])}],
    )

    # 3) 웨이포인트 로더 (시뮬/실차 공통) - f1tenth_waypoints 패키지
    waypoints_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('f1tenth_waypoints'),
                         'launch', 'waypoints.launch.py')
        ),
        launch_arguments={'map_name': map_name}.items(),
    )

    # 4) 컨트롤러 (MPPI, 조원 담당) - 시뮬/실차 공통, /scan /odom /global_path -> /drive
    controller_node = Node(
        package='f1tenth_controller_interface',
        executable='controller_node',
        name='mppi_controller',
        output='screen',
    )

    # 5) 차량 인터페이스만 스왑
    sim_vehicle_node = Node(
        package='f1tenth_vehicle_interface',
        executable='sim_vehicle_node',
        name='vehicle_interface',
        output='screen',
        condition=IfCondition(sim),
    )
    real_vehicle_node = Node(
        package='f1tenth_vehicle_interface',
        executable='real_vehicle_node',
        name='vehicle_interface',
        output='screen',
        condition=UnlessCondition(sim),
    )

    return LaunchDescription([
        sim_arg,
        map_name_arg,
        map_server_node,
        lifecycle_manager,
        localization_node,
        waypoints_launch,
        controller_node,
        sim_vehicle_node,
        real_vehicle_node,
    ])


def _pkg_exists(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False

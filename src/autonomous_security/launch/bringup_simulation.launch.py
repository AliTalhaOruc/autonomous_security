import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction # TimerAction eklendi
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_my_share = get_package_share_directory('autonomous_security')

    # Parametre Yollarını Kesin Olarak Tanımlayalım
    ekf_config = os.path.join(pkg_my_share, 'config', 'ekf.yaml')
    imu_config = os.path.join(pkg_my_share, 'config', 'imu_filter.yaml')
    slam_params = os.path.join(pkg_my_share, 'config', 'husky_slam_params.yaml')
    nav2_params = os.path.join(pkg_my_share, 'config', 'husky_nav.yaml')

    # Temiz ve tekil bir sözlük tanımı
    use_sim_time = {'use_sim_time': True}

    # 1. IMU Filter Node (Sözdizimi hatası düzeltildi)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[imu_config, use_sim_time], # Üst üste süslü parantez kaldırıldı
        remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]
    )

    # 2. EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, use_sim_time],
        remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]
    )

    # 3. SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, use_sim_time]
    )

    # 4. Nav2 Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'use_composition': 'False',
            'slam': 'True'
        }.items()
    )

    start_nav2_with_delay = TimerAction(
        period=8.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        imu_filter_node,
        ekf_node,
        slam_node,
        start_nav2_with_delay 
    ])


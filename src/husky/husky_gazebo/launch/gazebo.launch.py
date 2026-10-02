from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
pkg_my_share = get_package_share_directory('autonomous_security')
default_world_path = os.path.join(pkg_my_share, 'world', 'mili.world')
ARGUMENTS = [
    DeclareLaunchArgument(
        'world_path',
        default_value=default_world_path,
        description='The military world path'
    ),
]

def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable(
                'GAZEBO_MODEL_PATH',
                default_value=''
            ),
            '/usr/share/gazebo-11/models/:',
            str(Path(
                get_package_share_directory('husky_description')
            ).parent.resolve()),
            ':',
            os.path.join(pkg_my_share, 'models')
        ]
    )
    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
   # Gazebo'yu standart yöntemle başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'use_sim_time': 'true', # Saati otomatik halleder
            'pause': 'false'
        }.items()
    )
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=[
            '-entity', 'husky',
            '-topic', 'robot_description',
            '-x', '-17.0',
            '-y', '0.0',
            '-z', '0.4',
            '-Y', '0.0'
        ],
        output='screen',
    )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)

    return ld

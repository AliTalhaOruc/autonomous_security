from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Sadece ve sadece senin explorer Python düğümünü başlatan temiz launch
    explorer = Node(
        package='autonomous_security',
        executable='explorer',
        name='explorer_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        explorer
    ])

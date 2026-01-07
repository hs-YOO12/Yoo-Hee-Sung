import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time')

    params = os.path.join(
        get_package_share_directory('inpipe_robot'),
        'config',
        'joystick.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[params]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[params],
        # remappings=[('/cmd_vel', '/ipir_6dof_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])

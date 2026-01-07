import os
import xacro

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration #
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('inpipe_robot')
    launch_dir = os.path.join(share_dir, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim') #

    xacro_file = os.path.join(share_dir, 'urdf', 'ipir_joint.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true') #
    x_pose = LaunchConfiguration('x_pose', default='-0.6') #
    y_pose = LaunchConfiguration('y_pose', default='0.0') #
    z_pose = LaunchConfiguration('z_pose', default='0.075') #

    world = os.path.join( #
        share_dir,
        'worlds',
        'demo_world.world'
        # 'empty_world.world'
    )
    print('world model: {}', world)
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(share_dir, 'models')
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_dir, 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }.items()
    )

    jsb_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    spawn_ipir = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_dir, 'spawn_ipir.launch.py')
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'robot_description':robot_description
        }.items()
    )

    controller_wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ipir_wheel_controller", "--controller-manager-timeout", "100"],
        output="screen"
    )

    controller_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ipir_joint_position_controller", "--controller-manager-timeout", "100"],
        output="screen"
    )

    delayed_controller_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_node,
            on_exit=[TimerAction(period=2.0, actions=[controller_wheel_spawner, controller_position_spawner])]
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        set_env_vars_resources,
        gz_server,
        gz_client,
        spawn_ipir,
        rsp,
        jsb_node,
        # rviz_node
        delayed_controller_spawn,
    ])

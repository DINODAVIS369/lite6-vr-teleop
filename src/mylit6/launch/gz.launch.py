from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'mylit6'
    robot_name = 'mylit6_robot'
    urdf_file = 'mylite6test.urdf'
    controller_file = 'lite6_controllers.yaml'

    # --- Paths ---
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)
    controller_path = os.path.join(pkg_share, 'config', controller_file)

    # --- Read URDF content ---
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # --- Gazebo Harmonic ---
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf', '--verbose'],
        output='screen'
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # --- Spawn Robot ---
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    # --- Bridge Clock from Gazebo ---
    gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )



    # --- Controller Spawners (delayed) ---
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    arm_controller_spawner = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    # --- Return LaunchDescription ---
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,

        gz_clock_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])

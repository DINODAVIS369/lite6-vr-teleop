#!/usr/bin/env python3
"""
Lite6 VR Servo Launch File
===========================
Starts MoveIt Servo for the UFactory Lite6 arm and the VR controller
input bridge node.

Usage
-----
  # Simulation (fake hardware):
  ros2 launch mylit6 lite6_vr_servo.launch.py

  # Real robot (requires robot connected via Ethernet):
  ros2 launch mylit6 lite6_vr_servo.launch.py is_sim:=false robot_ip:=192.168.1.xxx

Arguments
---------
  is_sim     (bool)   – true = fake/simulation mode, false = real robot (default: true)
  robot_ip   (string) – IP of the physical Lite6 arm, required when is_sim:=false
  add_gripper (bool)  – include the Lite6 gripper in the model (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Declare arguments ────────────────────────────────────────────────────
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Use fake/simulated hardware (true) or real robot (false)'
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.200',
        description='IP address of the physical Lite6 arm (only used when is_sim:=false)'
    )
    add_gripper_arg = DeclareLaunchArgument(
        'add_gripper',
        default_value='false',
        description='Attach the Lite6 gripper to the model'
    )
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale',
        default_value='1.0',
        description='Linear velocity scale for VR input'
    )
    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale',
        default_value='1.0',
        description='Angular velocity scale for VR input'
    )
    vr_input_topic_arg = DeclareLaunchArgument(
        'vr_input_topic',
        default_value='/vr/controller/right/pose',
        description='Topic where the VR controller publishes its PoseStamped'
    )

    is_sim       = LaunchConfiguration('is_sim')
    robot_ip     = LaunchConfiguration('robot_ip')
    add_gripper  = LaunchConfiguration('add_gripper')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale= LaunchConfiguration('angular_scale')
    vr_input_topic = LaunchConfiguration('vr_input_topic')

    # ── MoveIt Servo (FAKE / simulation) ─────────────────────────────────────
    servo_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_servo'),
                'launch',
                'lite6_moveit_servo_fake.launch.py'
            ])
        ),
        launch_arguments={
            'add_gripper': add_gripper,
        }.items(),
        condition=IfCondition(is_sim),
    )

    # ── MoveIt Servo (REAL hardware) ─────────────────────────────────────────
    servo_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_servo'),
                'launch',
                'lite6_moveit_servo_realmove.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip':    robot_ip,
            'add_gripper': add_gripper,
            'report_type': 'dev',
        }.items(),
        condition=UnlessCondition(is_sim),
    )

    # ── VR Input Bridge Node ─────────────────────────────────────────────────
    vr_node = Node(
        package='mylit6',
        executable='vr_servo_input',
        name='vr_servo_input',
        output='screen',
        parameters=[{
            'linear_scale':    linear_scale,
            'angular_scale':   angular_scale,
            'command_frame':   'link_base',
            'filter_deadband': 0.005,
            'input_topic':     vr_input_topic,
            'output_topic':    '/servo_server/delta_twist_cmds',
        }],
    )

    return LaunchDescription([
        is_sim_arg,
        robot_ip_arg,
        add_gripper_arg,
        linear_scale_arg,
        angular_scale_arg,
        vr_input_topic_arg,
        servo_fake,
        servo_real,
        vr_node,
    ])

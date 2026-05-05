#!/usr/bin/env python3
"""
Lite6 VR Gazebo Servo Launch File (Fixed)
=========================================
Starts Gazebo Sim, MoveIt Servo, and VR Input with proper MoveIt parameters.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import load_yaml

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactry')
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    
    # 1. Build MoveIt Configs (This loads SRDF, URDF, Kinematics, etc.)
    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers', # Gazebo uses its own, but we need the model
        robot_type='lite',
        dof=6,
        prefix=prefix,
        hw_ns=hw_ns,
        add_gripper=add_gripper,
    ).to_moveit_configs()

    # 2. Gazebo Simulation
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_gazebo'), 'launch', 'lite6_beside_table_gazebo.launch.py'
        ])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'add_gripper': add_gripper,
            'load_controller': 'true',
            'show_rviz': 'false', # We will start our own RViz
        }.items(),
    )

    # 3. MoveIt Servo Node
    servo_yaml = load_yaml('xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo_yaml['move_group_name'] = 'lite6'
    servo_yaml['command_out_topic'] = '/lite6_traj_controller/joint_trajectory'
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_server',
        output='screen',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ],
    )

    # 4. RViz (Configured for Servo)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('xarm_moveit_servo'), 'rviz', 'servo.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True}
        ],
    )

    # 5. VR Input Node
    vr_input_node = Node(
        package='mylit6',
        executable='vr_servo_input',
        name='vr_servo_input',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'linear_scale': 1.0},
            {'angular_scale': 1.0},
        ],
    )

    # 6. Quest Bridge (UDP Receiver)
    quest_bridge_node = Node(
        package='mylit6',
        executable='quest_bridge.py',
        name='quest_bridge',
        output='screen',
        parameters=[{
            'port': 5005,
            'output_topic': '/vr/controller/right/pose'
        }]
    )

    return [
        gazebo_sim_launch,
        servo_node,
        rviz_node,
        vr_input_node,
        quest_bridge_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

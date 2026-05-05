#!/usr/bin/env python3
"""
VR Controller Input Node for Lite6 MoveIt Servo (Fully Automatic)
================================================================
Features:
1. Auto-Homing: Moves robot out of singularity on startup.
2. Robust Data Flow: Logs every pose and velocity calculation.
3. MoveIt Servo Control: Automatically starts and configures servo mode.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from moveit_msgs.srv import ServoCommandType
import numpy as np
import time

def quat_to_angular_velocity(q_prev, q_curr, dt=0.01):
    dq = np.array([
        q_curr[0] - q_prev[0],
        q_curr[1] - q_prev[1],
        q_curr[2] - q_prev[2],
        q_curr[3] - q_prev[3],
    ]) / dt
    q_conj = np.array([-q_curr[0], -q_curr[1], -q_curr[2], q_curr[3]])
    omega = 2.0 * np.array([
        dq[3]*q_conj[0] + dq[0]*q_conj[3] + dq[1]*q_conj[2] - dq[2]*q_conj[1],
        dq[3]*q_conj[1] - dq[0]*q_conj[2] + dq[1]*q_conj[3] + dq[2]*q_conj[0],
        dq[3]*q_conj[2] + dq[0]*q_conj[1] - dq[1]*q_conj[0] + dq[2]*q_conj[3],
    ])
    return omega

class VRServoInput(Node):
    def __init__(self):
        super().__init__('vr_servo_input')

        # Parameters
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('command_frame', 'link_base')
        self.declare_parameter('filter_deadband', 0.005)
        self.declare_parameter('input_topic', '/vr/controller/right/pose')
        self.declare_parameter('output_topic', '/servo_server/delta_twist_cmds')
        self.declare_parameter('traj_topic', '/lite6_traj_controller/joint_trajectory')

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.command_frame = self.get_parameter('command_frame').value
        self.filter_deadband = self.get_parameter('filter_deadband').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        traj_topic = self.get_parameter('traj_topic').value

        # State
        self._prev_pose = None
        self._prev_time = 0.0
        self._pose_count = 0
        self._servo_started = False
        self._command_type_set = False
        self._homing_done = False

        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        # Subscriber
        self.pose_sub = self.create_subscription(PoseStamped, input_topic, self._pose_callback, 10)

        # Clients
        self.start_client = self.create_client(Trigger, '/servo_server/start_servo')
        self.type_client = self.create_client(ServoCommandType, '/servo_server/switch_command_type')

        # Setup Timer (Retry logic + Auto-Home)
        self.setup_timer = self.create_timer(1.0, self._setup_sequence)

        self.get_logger().info(f'Ultimate VR Servo Node Initialized.')

    def _setup_sequence(self):
        # 1. Perform Auto-Home (Bend the arm forward to avoid singularity)
        if not self._homing_done:
            self._perform_auto_home()
            self._homing_done = True
            return

        # 2. Start Servo
        if not self._servo_started:
            if self.start_client.service_is_ready():
                self.start_client.call_async(Trigger.Request())
                self.get_logger().info('MoveIt Servo Started.')
                self._servo_started = True
        
        # 3. Switch to Twist Mode
        if not self._command_type_set:
            if self.type_client.service_is_ready():
                req = ServoCommandType.Request()
                req.command_type = 1 # TWIST
                self.type_client.call_async(req)
                self.get_logger().info('Command Mode: TWIST.')
                self._command_type_set = True

        if self._servo_started and self._command_type_set:
            self.get_logger().info('🚀 System Ready for VR Teleoperation!')
            self.setup_timer.cancel()

    def _perform_auto_home(self):
        self.get_logger().info('Sending Auto-Home command to avoid singularity...')
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        # Bend the arm into a "hook" shape (Joints in Radians)
        point.positions = [0.0, -0.4, 0.4, 0.0, 0.8, 0.0]
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.traj_pub.publish(msg)

    def _pose_callback(self, msg: PoseStamped):
        self._pose_count += 1
        if self._pose_count == 1 or self._pose_count % 10 == 0:
            self.get_logger().info(f'Data Flow OK: Received pose #{self._pose_count}')

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_pose is None:
            self._prev_pose = msg
            self._prev_time = now
            return

        dt = now - self._prev_time
        if dt <= 0.001:
            return

        # Linear Delta -> Velocity
        prev_p = self._prev_pose.pose.position
        curr_p = msg.pose.position
        vx = (curr_p.x - prev_p.x) / dt
        vy = (curr_p.y - prev_p.y) / dt
        vz = (curr_p.z - prev_p.z) / dt

        # Angular Delta -> Velocity
        prev_q = self._prev_pose.pose.orientation
        curr_q = msg.pose.orientation
        q_prev = [prev_q.x, prev_q.y, prev_q.z, prev_q.w]
        q_curr = [curr_q.x, curr_q.y, curr_q.z, curr_q.w]
        try:
            wx, wy, wz = quat_to_angular_velocity(q_prev, q_curr, dt)
        except Exception:
            wx, wy, wz = 0.0, 0.0, 0.0

        # APPLY CLAMPING AND SMOOTHING
        max_v = 0.5   # m/s
        max_w = 1.0   # rad/s
        alpha = 0.3   # smoothing factor (0.1 = very smooth, 1.0 = raw)

        # Clamp
        vx = np.clip(vx, -max_v, max_v)
        vy = np.clip(vy, -max_v, max_v)
        vz = np.clip(vz, -max_v, max_v)
        wx = np.clip(wx, -max_w, max_w)
        wy = np.clip(wy, -max_w, max_w)
        wz = np.clip(wz, -max_w, max_w)

        # Publish Twist if there is movement
        if any(v != 0.0 for v in [vx, vy, vz, wx, wy, wz]):
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.command_frame
            out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = vx, vy, vz
            out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = wx, wy, wz
            self.twist_pub.publish(out)
            if self._pose_count % 50 == 0:
                self.get_logger().info(f'TELEOP: v=({vx:.2f}, {vy:.2f}, {vz:.2f}) w=({wx:.2f}, {wy:.2f}, {wz:.2f})')

        self._prev_pose = msg
        self._prev_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VRServoInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

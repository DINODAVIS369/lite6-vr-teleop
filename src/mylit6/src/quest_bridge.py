#!/usr/bin/env python3
"""
Quest 3 Hand Tracking Bridge (Full 6-DOF Teleop)
==============================================
Maps both Position AND Orientation from Unity to ROS.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import threading
import math

class QuestBridge(Node):
    def __init__(self):
        super().__init__('quest_bridge')
        
        self.declare_parameter('port', 5005)
        self.declare_parameter('output_topic', '/vr/controller/right/pose')
        self.declare_parameter('scale', 0.8)
        
        self.port = self.get_parameter('port').value
        self.output_topic = self.get_parameter('output_topic').value
        self.scale = self.get_parameter('scale').value
        
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.get_logger().info('6-DOF MODE: Unlocking Orientation...')
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop)
        self.thread.daemon = True
        self.thread.start()

    def _receive_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = sock.recvfrom(4096)
                full_text = data.decode('utf-8', errors='ignore')
                lines = full_text.split('\n')
                
                for line in lines:
                    clean_line = line.strip()
                    if clean_line.startswith('Right wrist:'):
                        parts = [p.strip() for p in clean_line.split(',')]
                        
                        if len(parts) >= 8:
                            msg = PoseStamped()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = 'link_base'
                            
                            try:
                                # POSITION (Unity -> ROS FLU)
                                ux = float(parts[1]) # Right
                                uy = float(parts[2]) # Up
                                uz = float(parts[3]) # Forward
                                
                                msg.pose.position.x = (uz + 2.5) * self.scale
                                msg.pose.position.y = (-ux) * self.scale
                                msg.pose.position.z = (uy - 0.7) * self.scale + 0.2
                                
                                # ORIENTATION (Unity -> ROS FLU)
                                # Unity: X-Right, Y-Up, Z-Forward
                                # ROS:   X-Fwd,   Y-Left, Z-Up
                                # Quaternion components need to be remapped
                                qx = float(parts[4])
                                qy = float(parts[5])
                                qz = float(parts[6])
                                qw = float(parts[7])
                                
                                # Remap Unity (x,y,z,w) to ROS (x,y,z,w)
                                # This is a heuristic based on the axis mapping above
                                msg.pose.orientation.x = qz
                                msg.pose.orientation.y = -qx
                                msg.pose.orientation.z = qy
                                msg.pose.orientation.w = qw
                                
                                self.publisher.publish(msg)
                            except:
                                pass
            except socket.timeout:
                continue
            except Exception as e:
                break
        sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = QuestBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

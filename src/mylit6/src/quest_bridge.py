#!/usr/bin/env python3
"""
Quest 3 Hand Tracking Bridge (Wrist Mode)
========================================
Tracks the Wrist for precise 1:1 mirroring.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import threading

class QuestBridge(Node):
    def __init__(self):
        super().__init__('quest_bridge')
        
        self.declare_parameter('port', 5005)
        self.declare_parameter('output_topic', '/vr/controller/right/pose')
        self.declare_parameter('scale', 1.0)
        
        self.port = self.get_parameter('port').value
        self.output_topic = self.get_parameter('output_topic').value
        self.scale = self.get_parameter('scale').value
        
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.get_logger().info('WRIST MODE: 1:1 Mirroring Active')
        
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
                                # UNITY -> ROS (1:1)
                                ux = float(parts[1])
                                uy = float(parts[2])
                                uz = float(parts[3])
                                
                                # POSITION MAPPING
                                msg.pose.position.x = (uz + 2.8) * self.scale
                                msg.pose.position.y = (-ux) * self.scale
                                msg.pose.position.z = (uy - 0.7) * self.scale + 0.3
                                
                                # ORIENTATION MAPPING
                                uqx = float(parts[4])
                                uqy = float(parts[5])
                                uqz = float(parts[6])
                                uqw = float(parts[7])
                                
                                # Remap Unity -> ROS
                                msg.pose.orientation.x = uqz
                                msg.pose.orientation.y = -uqx
                                msg.pose.orientation.z = uqy
                                msg.pose.orientation.w = uqw
                                
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

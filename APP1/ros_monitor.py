# ROS Monitor
# This node :
# - listens for remote information requests (commands) on a TCP port
# - broadcasts the robot's position on a UDP port
# Team 12 - Edouard Barrimo, Xavier Gervais, Sami Ghoul-Duclos, Étienne Renaud

import rclpy
from rclpy.node import Node
import socket
import threading
from struct import *
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

HOST = "10.0.1.1"
BROADCAST = "10.0.1.255"

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class ROSMonitor(Node):
    def __init__(self):
        super().__init__('ros_monitor')
        # Add your subscriber here (odom, lidar, etc)
        self.sub_laser = self.create_subscription(LaserScan, "/scan", self.laser_callback, 0)
        self.sub_odo = self.create_subscription(Odometry, "/odom", self.odometry_callback, 0)

        # Current robot state:
        self.id = "10.0.1.1"
        self.pos = (0,0,0)
        self.obstacle = False

        # Params
        self.remote_request_port = self.declare_parameter('remote_request_port', 65432).value
        self.pos_broadcast_port = self.declare_parameter('pos_broadcast_port', 65431).value

        # Thread for RemoteRequest handling
        self.rr_thread = threading.Thread(target=self.rr_service)

        # Thread for Position Broadcasting
        self.pb_thread = threading.Thread(target=self.pb_service)

        # Start the threads
        self.rr_thread.start()
        self.pb_thread.start()

    def rr_service(self):
        self.get_logger().info(f"Beginning RemoteRequest service")

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 	
            s.bind((HOST, self.remote_request_port))

            while rclpy.ok():
                # Listen for incoming connections
                # Will execute again if connexion is broken
                s.listen(1)
                self.get_logger().info(f"Listening for remote requests on port {self.remote_request_port}")
                conn, addr = s.accept()
                
                with conn:
                    self.get_logger().info(f"Connected to {addr}")

                    # Reply loop, breaks when connexion is closed
                    while True:
                        # Receives the command
                        data = conn.recv(1024)

                        # Breaks reply loop if socket is closed by client
                        if not data:
                            self.get_logger().info(f"Connexion closed by client. Dissconnecting...")
                            conn.close()
                            break

                        # Decodes the command and uses handle_command() to reply
                        command = data.decode().strip()
                        response = self.handle_command(command)
                        conn.sendall(response.encode())

    def pb_service(self):
    	self.get_logger().info(f"Beginning PositionBroadcast service")

    	with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while True:
                # Update the position data (TBD)
                x, y, theta = self.pos
                message = f"{x}, {y}, {theta}, {self.id}"

                #Try sending the message through the socket
                try:
                    s.sendto(message.encode(), (BROADCAST, self.pos_broadcast_port))
                    self.get_logger().info(message)
                except PermissionError as e:
                    self.get_logger().error(f"Permission Error in PositionBroadcast: {e}")

                # Wait for a second
                time.sleep(1)

    def handle_command(self, command):
        if command == "RPOS":
            x, y, theta = self.pos
            return f"Position: {x}, {y}, {theta}"
        elif command == "OBSF":
            return f"Obstacle: {self.obstacle}"
        elif command == "RBID":
            return f"Robot ID: {self.id}"
        else:
            return "Unknown command"

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pos = (x, y, theta)

    def laser_callback(self, msg):
        # Check if there is an obstacle in the lidar scan's range
        sorted_ranges = sorted(msg.ranges)
        self.obstacle = (sorted_ranges[4] < 1.0)

def main(args=None):
    rclpy.init(args=args)
    node = ROSMonitor()
    rclpy.spin(node)
    self.sub_odo.destroy()
    rclpy.shutdown()

if __name__=="__main__":
    main()

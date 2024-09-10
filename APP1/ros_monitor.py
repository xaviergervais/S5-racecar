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
#from tf_transformations import euler_from_quaternion

HOST = "10.0.1.1"
BROADCAST = "10.42.0.255"

# example:
#   python ros_monitor.py scan:=/racecar/scan odom:=/racecar/odometry/filtered

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

def ip_to_uint32(ip):
    return unpack("!I", socket.inet_aton(ip))[0]

class ROSMonitor(Node):
    def __init__(self):
        super().__init__('ros_monitor')
        # Add your subscriber here (odom, lidar, etc)
        #self.sub_laser = self.create_subscription(Laser, "/scan", self.position_broadcast_service, 0)
        #self.sub_odo = self.create_subscription(Odometry, "/odometry/filtered", self.position_broadcast_service, 0)

        # Current robot state:
        self.id = "10.0.1.1"
        self.pos = (0,0,0)
        self.obstacle = False

        # Params
        self.remote_request_port = self.declare_parameter('remote_request_port', 65432).value
        self.pos_broadcast_port = self.declare_parameter('pos_broadcast_port', 65431).value

        # Thread for RemoteRequest handling
        self.rr_thread = threading.Thread(target=self.rr_loop)

        # Thread for Position Broadcasting
        self.pb_thread = threading.Thread(target=self.position_broadcast_service)

        self.get_logger().info("ROSMonitor started.")

        # Start the threads
        self.rr_thread.start()
        self.pb_thread.start()

    def rr_loop(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 	
            s.bind((HOST, self.remote_request_port))
            s.listen()
            self.get_logger().info(f"Listening for remote requests on port {self.remote_request_port}")
            while rclpy.ok():
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        command = data.decode().strip()
                        response = self.handle_command(command)
                        conn.sendall(response.encode())

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

    def position_broadcast_service(self):
    	self.get_logger().info(f"1")
    	ip_address = ip_to_uint32(BROADCAST)
    	self.get_logger().info(f"2")
    	with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while True:
                # Broadcast the position
                x, y, theta = self.pos
                message = f"{x}, {y}, {theta}, {ip_address}"
                try:
                    s.sendto(message.encode(), (BROADCAST, self.pos_broadcast_port))
                    self.get_logger().info(message)
                except PermissionError as e:
                    self.get_logger().error(f"PermissionError: {e}")
                time.sleep(1)  # Broadcast every second

def main(args=None):
    rclpy.init(args=args)
    node = ROSMonitor()
    rclpy.spin(node)
    rclpy.shutdown()   

if __name__=="__main__":
    main()



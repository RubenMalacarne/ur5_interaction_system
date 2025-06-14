#!/usr/bin/env python3
"""
@file socket_listener.py
@brief ROS 2 node to test communication socket.
"""
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SocketServer(Node):
    def __init__(self):
        super().__init__('socket_server_node')
        self.publisher_ = self.create_publisher(String, 'porta_comando', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('0.0.0.0', 12345))
        self.sock.listen(1)
        self.get_logger().info("🔌 listen port porta 12345...")

        self.create_timer(0.1, self.accept_connection)

    def accept_connection(self):
        self.sock.settimeout(0.01)
        try:
            conn, addr = self.sock.accept()
            data = conn.recv(1024).decode().strip()
            self.get_logger().info(f"📩 Recived: {data}")
            if data == "SIGNAL":
                msg = String()
                msg.data = "chiudi"
                self.publisher_.publish(msg)
                self.get_logger().info("📤 publish command on 'porta_comando'")
            conn.close()
        except socket.timeout:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SocketServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
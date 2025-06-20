#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal
import socket
import threading

class EmergencyStopWatcher(Node):
    def __init__(self):
        super().__init__('emergency_stop_watcher')

        self.process = None
        self.system_running = False
        self.current_stop_state = False

        self.publisher_ = self.create_publisher(Bool, '/emergency_stop', 10)
        self.sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_callback,
            10
        )
        # to start system without waiting check emergency stop botton
        # self.start_system()

        self.server_thread = threading.Thread(target=self.start_socket_server, daemon=True)
        self.server_thread.start()

    def start_socket_server(self):
        HOST = '0.0.0.0'
        PORT = 12345

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(1)
            self.get_logger().info(f"🔌 Socket ACTIVATED on: {HOST}:{PORT}")
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(1024).decode().strip().upper()
                    self.get_logger().info(f"📩 RECIVED: {data}")
                    if data == "FALSE":
                        self.current_stop_state = True  # Toggle
                        msg = Bool()
                        msg.data = self.current_stop_state
                        self.publisher_.publish(msg)
                        stato_str = "STOP" if self.current_stop_state else "AVVIO"
                        self.get_logger().info(f"🔁 Stato togglato: {stato_str}")
                    elif data == "TRUE":
                        self.current_stop_state = False
                        msg = Bool()
                        msg.data = self.current_stop_state
                        self.publisher_.publish(msg)
                        stato_str = "STOP" if self.current_stop_state else "AVVIO"
                        self.get_logger().info(f"🔁 Stato togglato: {stato_str}")
                    else:
                        self.get_logger().warn(f"⚠️ Comando sconosciuto: {data}")

    def start_system(self):
        if self.process is None or self.process.poll() is not None:
            self.process = subprocess.Popen(
                ["ros2", "launch", "cr_bringup", "system_bringup.launch.py"]
            )
            self.system_running = True
            self.get_logger().info("✅ START SYSTEM.")

    def stop_system(self):
        if self.process is not None and self.process.poll() is None:
            self.get_logger().warn("🛑 STOP SISTEMS WAIT...")
            self.process.send_signal(signal.SIGINT)
            self.process.wait()
            self.get_logger().info("☠️ SYSTEM KILLED.")
        self.system_running = False

    def emergency_callback(self, msg):
        if msg.data:
            if self.system_running:
                self.stop_system()
            else:
                self.get_logger().warn("⚠️ Recived stop but sistem is alreadt stopped.")
        else:
            if not self.system_running:
                self.get_logger().info("🔁 REBOOT SYSTEMS.")
                self.start_system()
            else:
                self.get_logger().info("ℹ️ Reboot ignored, system is already running.")

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
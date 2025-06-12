#!/usr/bin/env python3
import rclpy, copy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer

class MirrorCameraNode(Node):
    def __init__(self):
        super().__init__("mirror_camera_node")

        self.bridge = CvBridge()
        self.fx = 525.0
        
        # Contatori per debug
        self.rgb_count = 0
        self.depth_count = 0
        self.sync_count = 0

        # --------- subscriber singoli per debug ----------
        #self.rgb_debug_sub = self.create_subscription(Image, "/coppelia_camera/rgb", self.rgb_debug_cb, 10)
        #self.depth_debug_sub = self.create_subscription(Image, "/coppelia_camera/depth", self.depth_debug_cb, 10)

        # --------- subscriber con message_filters ----------
        qos = rclpy.qos.QoSProfile(depth=10)
        self.rgb_sub = Subscriber(self, Image, "/coppelia_camera/rgb", qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, "/coppelia_camera/depth", qos_profile=qos)

        # Prova prima con ApproximateTimeSynchronizer (più permissivo)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 
                                              queue_size=10, slop=0.1)
        self.sync.registerCallback(self.rgb_depth_cb)

        # --------- publisher ----------
        self.rgb_pub = self.create_publisher(Image, "cr_vision/mirrored_camera/rgb", 10)
        self.depth_pub = self.create_publisher(Image, "cr_vision/mirrored_camera/depth", 10)

        # Timer per log periodico
        #self.debug_timer = self.create_timer(2.0, self.debug_status)

        #self.get_logger().info("MirrorCameraNode with RGB-Depth sync ready.")

    # def rgb_debug_cb(self, msg):
    #     self.rgb_count += 1
    #     if self.rgb_count % 10 == 1:  # Log ogni 10 messaggi
    #         self.get_logger().info(f"RGB #{self.rgb_count}: stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, frame_id='{msg.header.frame_id}', size={msg.width}x{msg.height}")

    # def depth_debug_cb(self, msg):
    #     self.depth_count += 1
    #     if self.depth_count % 10 == 1:  # Log ogni 10 messaggi
    #         self.get_logger().info(f"Depth #{self.depth_count}: stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, frame_id='{msg.header.frame_id}', size={msg.width}x{msg.height}")

    # def debug_status(self):
    #     self.get_logger().info(f"Status: RGB={self.rgb_count}, Depth={self.depth_count}, Sync={self.sync_count}")

    def rgb_depth_cb(self, rgb_msg: Image, depth_msg: Image):
        self.sync_count += 1
        
        # Log della sincronizzazione
        rgb_stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        time_diff = abs(rgb_stamp - depth_stamp)
        
        #self.get_logger().info(f"SYNC #{self.sync_count}: time_diff={time_diff*1000:.2f}ms, RGB_frame='{rgb_msg.header.frame_id}', Depth_frame='{depth_msg.header.frame_id}'")

        try:
            # ---- mirror RGB ----
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            rgb_flipped = cv2.flip(rgb, 1)
            rgb_out = self.bridge.cv2_to_imgmsg(rgb_flipped, encoding="rgb8")
            rgb_out.header = rgb_msg.header
            self.rgb_pub.publish(rgb_out)

            # ---- mirror Depth ----
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            depth_flipped = cv2.flip(depth, 1)
            depth_out = self.bridge.cv2_to_imgmsg(depth_flipped, encoding="32FC1")
            depth_out.header = depth_msg.header
            self.depth_pub.publish(depth_out)
            
            #self.get_logger().info(f"Successfully published mirrored images #{self.sync_count}")
            
        except Exception as e:
            self.get_logger().error(f"Error in mirroring: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MirrorCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
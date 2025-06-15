#!/usr/bin/env python3
"""
@file mirror_scena.py
@brief Node to mirror RGB and Depth images from a camera in CoppeliaSim.
A ROS2 node that subscribes to RGB and Depth images, mirrors them horizontally,
and publishes the mirrored images to new topics.
"""
import rclpy, copy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer

class MirrorCameraNode(Node):
    """
    @file mirror_scena.py
    @brief Node to mirror RGB and Depth images from a camera in CoppeliaSim.
    A ROS2 node that subscribes to RGB and Depth images, mirrors them horizontally,
    and publishes the mirrored images to new topics.
    
    """
    def __init__(self):
        """
        @brief Constructor for MirrorCameraNode.
        Initializes publishers, subscribers, and the CvBridge for image conversion.
        """
        super().__init__("mirror_camera_node")

        self.bridge = CvBridge()
        self.fx = 525.0
        
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
        
        # Debug:
        # Contatori per debug
        self.rgb_count = 0
        self.depth_count = 0
        self.sync_count = 0

        # --------- subscriber per debug ----------
        #   use to understand the sync and how many messages are received
        
        #self.rgb_debug_sub = self.create_subscription(Image, "/coppelia_camera/rgb", self.rgb_debug_cb, 10)
        #self.depth_debug_sub = self.create_subscription(Image, "/coppelia_camera/depth", self.depth_debug_cb, 10)

        #self.debug_timer = self.create_timer(2.0, self.debug_status)
        #self.get_logger().info("MirrorCameraNode with RGB-Depth sync ready.")

    # --------- Debug functions ----------
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
        """
        @brief Callback for synchronized RGB and depth image messages.

        This function is triggered whenever a synchronized pair of RGB and depth images
        is received via message_filters. It mirrors (horizontally flips) both images and
        republishes them on the configured output topics.

        @param rgb_msg The incoming RGB image as a sensor_msgs.msg.Image.
        @param depth_msg The incoming depth image as a sensor_msgs.msg.Image.
        """
        self.sync_count += 1        
        rgb_stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        # Calculate the time difference between RGB and Depth messages
        time_diff = abs(rgb_stamp - depth_stamp)
        
        
        try:
            # ---- mirror RGB ---- 
            #convert message RGB in immage OpenCV
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            # Flip image horizontally
            rgb_flipped = cv2.flip(rgb, 1)
            # Convert back to ROS Image message
            rgb_out = self.bridge.cv2_to_imgmsg(rgb_flipped, encoding="rgb8")
            # Copy header (to preserve timestamp and frame_id)
            rgb_out.header = rgb_msg.header
            self.rgb_pub.publish(rgb_out)

            # ---- mirror Depth ---- (same things see above)
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
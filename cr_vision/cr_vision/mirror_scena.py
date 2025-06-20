#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import copy
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage



class MirrorCameraNode(Node):
    """
    @file mirror_scena.py
    @brief Mirror camera node for ROS2

    A ROS2 node that mirrors camera data from a simulation.

    This node:
    - Flips RGB and depth images horizontally
    - Mirrors PointCloud2 data based on intrinsic parameters
    - Mirrors transforms in TF messages (position + rotation reflection)

    Subscribed topics (INPUT):
    - /coppelia_camera/rgb (sensor_msgs/Image)
    - /coppelia_camera/depth (sensor_msgs/Image)
    - /coppelia_camera/pointcloud (sensor_msgs/PointCloud2)
    - /tf (tf2_msgs/TFMessage)

    Published topics (OUTPUT):
    - cr_vision/mirrored_camera/rgb
    - cr_vision/mirrored_camera/depth
    - cr_vision/mirrored_camera/pointcloud
    - cr_vision/mirrored/tf
    """
    def __init__(self):
        super().__init__('mirror_camera_node')
        
        self.bridge = CvBridge()

        #parameter camera:
        self.fx = 525.0
        self.desired_encoding_rgb = "rgb8"
        self.desired_encoding_depth = "32FC1"
        
        # Subscriber
        self.rgb_sub = self.create_subscription(
            Image, '/coppelia_camera/rgb', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/coppelia_camera/depth', self.depth_callback, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/coppelia_camera/pointcloud', self.pointcloud_callback, 10)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        
        # Publisher
        self.rgb_pub = self.create_publisher(Image, 'cr_vision/mirrored_camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, 'cr_vision/mirrored_camera/depth', 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'cr_vision/mirrored_camera/pointcloud', 10)
        self.tf_pub = self.create_publisher(TFMessage, 'cr_vision/mirrored/tf', 10)

        
        self.get_logger().info("mirror camera node activated.")


    def rgb_callback(self, msg):
        """
        Mirror the RGB image horizontally and republish it.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.desired_encoding_rgb)
        except Exception as e:
            self.get_logger().error(f"Conversion ERROR RGB: {e}")
            return
        # use file about opencv to mirror the image
        mirrored = cv2.flip(cv_image, 1)
        mirrored_msg = self.bridge.cv2_to_imgmsg(mirrored, encoding =self.desired_encoding_rgb)
        mirrored_msg.header = msg.header
        # (Se vuoi cambiare timestamp: mirrored_msg.header.stamp = self.get_clock().now().to_msg())
        self.rgb_pub.publish(mirrored_msg)


    def depth_callback(self, msg):
        """
        Mirror the depth image horizontally and republish it.
        """
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.desired_encoding_depth)
        except Exception as e:
            self.get_logger().error(f"Conversion ERROR Depth: {e}")
            return
        mirrored_depth = cv2.flip(cv_depth, 1)
        mirrored_msg = self.bridge.cv2_to_imgmsg(mirrored_depth, encoding =self.desired_encoding_depth)
        mirrored_msg.header = msg.header
        self.depth_pub.publish(mirrored_msg)

    def pointcloud_callback (self,msg):
        """
        Mirror the PointCloud2 data and apply X-axis translation.
        """
        try:
            # nota: l'immagine usata è 2d quindi la depth si rifa su di essa 
            height = msg.height
            width = msg.width
            num_points = height * width
            points = np.frombuffer(msg.data, dtype=np.float32).reshape((num_points, 3))
            
        except Exception as e:
            self.get_logger().error(f"Conversion ERROR PointCloud2: {e}")
            return
        
        points_organized = points.reshape((height, width, 3))
        points_flipped = np.flip(points_organized, axis=1)

        # mirrorino e traslazione lungo z termine --> z/fx
        points_flipped[:, :, 0] = -points_flipped[:, :, 0] - (points_flipped[:, :, 2] / self.fx)

        mirrored_points = points_flipped.reshape((num_points, 3))
        mirrored_data = mirrored_points.astype(np.float32).tobytes()

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = height
        new_msg.width = width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.data = mirrored_data
        new_msg.is_dense = msg.is_dense

        self.pc_pub.publish(new_msg)
        
    def tf_callback(self, msg):
        """
        Mirror transforms in the TF message and publish them.
        """
        tf_new = TFMessage()
        try:
            tf_msg = copy.deepcopy(msg)
            for transform in tf_msg.transforms:
                new_transform = copy.deepcopy(transform)
                # mirror translation and rotation
                new_transform.transform.translation.x = -transform.transform.translation.x
                new_transform.transform.rotation.y = -transform.transform.rotation.y
                new_transform.transform.rotation.z = -transform.transform.rotation.z
                # mirroring rotazione: R_new = M * R * M con M = diag([-1,1,1])
                q = [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ]
                R = self.quaternion_to_matrix(q)
                M = np.diag([-1, 1, 1])
                R_new = M @ R @ M
                q_new = self.matrix_to_quaternion(R_new)
                new_transform.transform.rotation.x = q_new[0]
                new_transform.transform.rotation.y = q_new[1]
                new_transform.transform.rotation.z = q_new[2]
                new_transform.transform.rotation.w = q_new[3]

                tf_new.transforms.append(new_transform)
        except Exception as e:
            self.get_logger().error(f"Errore conversione TF: {e}")
            return

        self.tf_pub.publish(tf_msg)


    def quaternion_to_matrix(self, q):
        """
        Convert a quaternion [x, y, z, w] to a 3x3 rotation matrix.
        """
        x, y, z, w = q
        R = np.array([
            [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        return R

    def matrix_to_quaternion(self, R):
        """
        Convert a 3x3 rotation matrix to a quaternion [x, y, z, w].
        """
        m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
        m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
        m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

        trace = m00 + m11 + m22
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m21 - m12) * s
            y = (m02 - m20) * s
            z = (m10 - m01) * s
        else:
            if (m00 > m11) and (m00 > m22):
                s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
                w = (m21 - m12) / s
                x = 0.25 * s
                y = (m01 + m10) / s
                z = (m02 + m20) / s
            elif m11 > m22:
                s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
                w = (m02 - m20) / s
                x = (m01 + m10) / s
                y = 0.25 * s
                z = (m12 + m21) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
                w = (m10 - m01) / s
                x = (m02 + m20) / s
                y = (m12 + m21) / s
                z = 0.25 * s
        return [x, y, z, w]
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MirrorCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import geometry_msgs.msg

import numpy as np

from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs

from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from cr_interfaces.msg import ObjectDetectionBox, ObjectDetectionResult


class ObjectDetectionNode(Node):
    """
    @file object_detection.py
    @brief YOLO-based object detection ROS 2 node with depth processing and TF publishing.

    This node:
    - Subscribes to RGB and depth images from a mirrored camera
    - Runs YOLO object detection on the RGB image with pre trainer model
    - Uses depth to compute 3D camera coordinates of detected objects
    - Transforms those coordinates to the world frame
    - Publishes detection results and annotated images
    - Publishes a TF frame per detected object
    """
    def __init__(self) -> None:
        super().__init__('object_detection_node')
        
        # QoS settings for the subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )
            
        package_share_directory = get_package_share_directory('cr_vision')
        #import the model
        model_path = os.path.join(package_share_directory, 'data', 'yolo_cubi.pt')
        self.model = YOLO(model_path)
        
        #subscribe to RGB and depth images
        self.rgb_subscription = self.create_subscription(
            Image,
            'cr_vision/mirrored_camera/rgb',
            self.rgb_callback,
            qos_profile
        )
        self.depth_subscription = self.create_subscription(
            Image,
            'cr_vision/mirrored_camera/depth',
            self.depth_callback,
            qos_profile
        )
        
        #publish the detection results
        self.image_publisher = self.create_publisher(Image, 'cr_vision/yolov8_detection_image', 1)
        self.detection_publisher = self.create_publisher(
            ObjectDetectionResult,
            'cr_vision/yolov8_detection_results',
            1
        )
        #publish the TF frames
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        self.depth_image = None
        self.get_logger().info("Object Detection Node with Depth and World Transform has been started.")

    def depth_callback(self, depth_data: Image) -> None:
        """
        Callback to store the latest depth image for 3D calculations.
        """
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def rgb_callback(self, rgb_data: Image) -> None:
        """
        Callback to process an RGB image and run YOLO detection.
        Publishes annotated image, detection results, and object transforms.
        """
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        results = self.model(cv_image)

        if len(results) > 0 and results[0].boxes is not None:
            
            boxes = results[0].boxes
            annotated_frame = results[0].plot()

            detection_msg = ObjectDetectionResult()
            detection_msg.header = Header()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = "object_detection"

            for i, box in enumerate(boxes):
                xyxy = box.xyxy[0]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                label = results[0].names[cls_id] if hasattr(results[0], 'names') else str(cls_id)

                x_min, y_min, x_max, y_max = xyxy
                cx = int((x_min + x_max) / 2.0)
                cy = int((y_min + y_max) / 2.0)

                x_3d, y_3d, z_3d, world_x, world_y, world_z, distance = self.yolo_detection(
                    cx, cy, x_min, y_max, annotated_frame, rgb_data
                )

                # self.get_logger().info(
                #     f"Objects[{i}]: label={label}, conf={conf:.2f}, bbox=({x_min:.1f},{y_min:.1f},{x_max:.1f},{y_max:.1f}), "
                #     f"center=({cx},{cy}), distance={distance:.3f}, 3D_cam=({x_3d},{y_3d},{z_3d})"
                # )

                cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)

                # Messaggio di detection
                box_msg = ObjectDetectionBox()
                box_msg.id = i
                box_msg.label = label
                box_msg.confidence = conf
                box_msg.x_min = float(x_min)
                box_msg.y_min = float(y_min)
                box_msg.x_max = float(x_max)
                box_msg.y_max = float(y_max)
                box_msg.distance = distance
                box_msg.world_x = world_x
                box_msg.world_y = world_y
                box_msg.world_z = world_z

                detection_msg.boxes.append(box_msg)

                self.publish_tf(world_x, world_y, world_z, f"{label}_{i}")

            self.detection_publisher.publish(detection_msg)
            
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_publisher.publish(annotated_msg)
        else:
            self.get_logger().info("I don't see any objects.")

    def yolo_detection(self, cx, cy, x_min, y_max, annotated_frame, rgb_data):
        """
        Computes 3D position in the camera and world frame from bounding box center.
        """
        x_3d = y_3d = z_3d = None
        world_x = world_y = world_z = 0.0
        label_3d = ""
        distance = -1.0

        if self.depth_image is not None:
            h, w = self.depth_image.shape
            if 0 <= cx < w and 0 <= cy < h:
                distance = float(self.depth_image[cy, cx])  
                if distance > 0: 
                    fx = 525.0  
                    fy = 525.0
                    cx_optical = w / 2.0
                    cy_optical = h / 2.0

                    z_3d = distance
                    x_3d = (cx - cx_optical) * z_3d / fx
                    y_3d = (cy - cy_optical) * z_3d / fy

                    label_3d = f"X: {x_3d:.2f}, Y: {y_3d:.2f}, Z: {z_3d:.2f} m"
                    cv2.putText(
                        annotated_frame,
                        label_3d,
                        (int(x_min), int(y_max) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                        2
                    )

                    # Convert in coordinate mondo
                    try:
                        camera_point = geometry_msgs.msg.PointStamped()
                        camera_point.header.stamp = rgb_data.header.stamp
                        camera_point.header.frame_id = "camera_rgbd"
                        camera_point.point.x = x_3d
                        camera_point.point.y = y_3d
                        camera_point.point.z = z_3d

                        transform = self.tf_buffer.lookup_transform(
                            'world',
                            'camera_rgbd',
                            rclpy.time.Time())

                        world_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                        world_x = world_point.point.x
                        world_y = world_point.point.y
                        world_z = world_point.point.z
                    except Exception as e:
                        self.get_logger().error(f"Transform error: {e}")

        return x_3d, y_3d, z_3d, world_x, world_y, world_z, distance

    def publish_tf(self, x_world: float, y_world: float, z_world: float, object_name: str) -> None:
        """
        Publishes a static transform (TF) for a detected object.
        """
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  # Frame di riferimento 
        t.child_frame_id = f"{object_name}_frame"
        
        t.transform.translation.x = x_world
        t.transform.translation.y = y_world
        t.transform.translation.z = z_world
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
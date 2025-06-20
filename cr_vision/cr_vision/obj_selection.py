#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cr_interfaces.msg import ObjectDetectionResult, ObjectInfoArray, ObjectInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjSelectionNode(Node):
        
    """
    @file obj_selection.py
    @brief Selects target objects from YOLO detections and publishes standardized object info.

    This ROS 2 node:
    - Subscribes to YOLO detection results and camera images
    - Filters detections based on target labels from parameters
    - Adds fixed size metadata to selected objects
    - Publishes the selected objects and an annotated image
    """
    def __init__(self):
        super().__init__('obj_selection_node')

        # parameters of the cube
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_labels', ["green_cube", "red_cube"]),
                ('target_size_x', 0.05),
                ('target_size_y', 0.05),
                ('target_size_z', 0.05)
            ]
        )
        
        self.target_labels = self.get_parameter('target_labels').get_parameter_value().string_array_value
        self.target_size_x = self.get_parameter('target_size_x').get_parameter_value().double_value
        self.target_size_y = self.get_parameter('target_size_y').get_parameter_value().double_value
        self.target_size_z = self.get_parameter('target_size_z').get_parameter_value().double_value

        # Subscriber 
        self.subscription = self.create_subscription(
            ObjectDetectionResult,
            'cr_vision/yolov8_detection_results',
            self.detection_callback,
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            'cr_vision/mirrored_camera/rgb', 
            self.image_callback,
            10
        )
        
        # Publisher
        self.obj_selected_pub_ = self.create_publisher(
            ObjectInfoArray,
            'cr_vision/object_selection_results',
            10
        )
        self.image_pub_ = self.create_publisher(
            Image,
            'cr_vision/object_selection_image',
            10
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
        
        self.get_logger().info(f"obj_selection_node avviato. Target da selezionare: '{self.target_labels}'.")

    def image_callback(self, msg):
        """
        Callback that stores the most recent camera image for later annotation.
        """
        self.latest_image = msg

    def detection_callback(self, detection_msg: ObjectDetectionResult):
        """
        Callback that filters detections by label and publishes ObjectInfoArray and an annotated image.
        @param detection_msg: Incoming detection results from YOLO.
        """
        if not detection_msg.boxes:
            return  

        if self.latest_image is None:
            self.get_logger().warn("Nessuna immagine disponibile, impossibile annotare.")
            return
        
        selected_objects = [] # Array di ObjectInfo
        annotated_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")

        for box in detection_msg.boxes:
            label = box.label
            obj_id = box.id
            
            # Check if the label is in the target labels
            if label in self.target_labels:
                self.get_logger().info(f"Rilevato '{label}' con id={obj_id}.")

                obj = ObjectInfo()
                obj.id = obj_id
                obj.label = label
                obj.center.x = box.world_x
                obj.center.y = box.world_y
                obj.center.z = box.world_z
                obj.size.x =  self.target_size_x
                obj.size.y = self.target_size_y
                obj.size.z = self.target_size_z

                selected_objects.append(obj)

                x_min, y_min, x_max, y_max = int(box.x_min), int(box.y_min), int(box.x_max), int(box.y_max)
                cv2.rectangle(annotated_image, (x_min, y_min), (x_max, y_max), (255, 0, 255), 2)
                cv2.putText(annotated_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        # Annotate the image with the detected objects
        if selected_objects:
            selected_msg = ObjectInfoArray()
            selected_msg.header = detection_msg.header  
            selected_msg.objects = selected_objects  

            self.obj_selected_pub_.publish(selected_msg)

            annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.image_pub_.publish(annotated_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjSelectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

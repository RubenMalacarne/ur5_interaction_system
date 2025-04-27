#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cr_interfaces.msg import ObjectDetectionResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjSelectionNode(Node):
    def __init__(self):
        super().__init__('obj_selection_node')
        
        # Dichiarazione parametro per la label da cercare
        self.declare_parameter('target_label', 'green_cube')
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value
        
        # Subscriber ai detection results
        self.subscription = self.create_subscription(
            ObjectDetectionResult,
            'cr_vision/yolov8_detection_results',
            self.detection_callback,
            10
        )
        
        # Subscriber all'immagine originale della camera
        self.image_subscription = self.create_subscription(
            Image,
            'cr_vision/mirrored_camera/rgb', 
            self.image_callback,
            10
        )
        
        self.obj_selected_pub_ = self.create_publisher(
            ObjectDetectionResult,
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
        
        self.get_logger().info(f"obj_selection_node avviato. Target da selezionare: '{self.target_label}'.")

    def image_callback(self, msg):
        self.latest_image = msg

    def detection_callback(self, detection_msg: ObjectDetectionResult):
        if not detection_msg.boxes:
            return  

        if self.latest_image is None:
            self.get_logger().warn("Nessuna immagine disponibile, impossibile annotare.")
            return
        
        selected_boxes = [] 
        annotated_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")

        for box in detection_msg.boxes:
            label = box.label
            obj_id = box.id

            if label == self.target_label:
                self.get_logger().info(f"Rilevato '{label}' con id={obj_id}.")
                selected_boxes.append(box)

                x_min, y_min, x_max, y_max = int(box.x_min), int(box.y_min), int(box.x_max), int(box.y_max)
                cv2.rectangle(annotated_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if selected_boxes:
            selected_msg = ObjectDetectionResult()
            selected_msg.header = detection_msg.header  
            selected_msg.boxes = selected_boxes  

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

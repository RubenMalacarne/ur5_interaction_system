#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cr_interfaces.msg import ObjectDetectionResult

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjSelectionNode(Node):
    def __init__(self):
        super().__init__('obj_selection_node')
        
        self.last_image = None

        # Subscription ai detection results
        self.detection_subscription = self.create_subscription(
            ObjectDetectionResult,
            'cr_vision/yolov8_detection_results',
            self.detection_callback,
            10
        )

        # Subscription all'immagine annotata
        self.image_subscription = self.create_subscription(
            Image,
            'cr_vision/yolov8_detection_image',
            self.image_callback,
            10
        )

        # Publishers
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
        self.get_logger().info("obj_selection_node avviato e in ascolto su /yolov8_detection_results e /yolov8_detection_image.")


    def image_callback(self, image_msg: Image):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting received image: {e}")



    def detection_callback(self, detection_msg: ObjectDetectionResult):
        if not detection_msg.boxes:
            return

        if self.last_image is None:
            self.get_logger().warn("No image received yet to annotate!")
            return

        selected_boxes = []
        annotated_image = self.last_image.copy()  # Copia l'immagine per disegnare sopra

        for box in detection_msg.boxes:
            label = box.label
            obj_id = box.id

            if label == "traffic light":
                self.get_logger().info(f"Rilevata 'traffic light' con id={obj_id}.")
                selected_boxes.append(box)

                x_min, y_min, x_max, y_max = int(box.x_min), int(box.y_min), int(box.x_max), int(box.y_max)
                cv2.rectangle(annotated_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if selected_boxes:
            selected_msg = ObjectDetectionResult()
            selected_msg.header = detection_msg.header
            selected_msg.boxes = selected_boxes

            self.obj_selected_pub_.publish(selected_msg)

            # Pubblica l'immagine annotata
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

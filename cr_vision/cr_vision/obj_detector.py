#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
from cr_interfaces.msg import ObjectInfoArray, ObjectInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import geometry_msgs.msg
import matplotlib.pyplot as plt
from std_msgs.msg import Header


class ObjectDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('object_detector_node')

        # ------------------------------------------------------------
        # 1) Gestione parametri e inizializzazioni di ROS 2 / YOLO / TF
        # ------------------------------------------------------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_labels', ['green_cube', 'red_cube', 'blue_cube']),
                ('target_size_x', 0.05),
                ('target_size_y', 0.05),
                ('target_size_z', 0.15),
                ('camera_width', 512),
                ('camera_height', 512),
                ('camera_fov_deg', 60)
            ]
        )

        self.target_labels = list(self.get_parameter('target_labels').value)
        self.target_size_x = self.get_parameter('target_size_x').value
        self.target_size_y = self.get_parameter('target_size_y').value
        self.target_size_z = self.get_parameter('target_size_z').value

        # ROI per filtrare i risultati di YOLO (opzionale)
        self.roi_x_min = 200
        self.roi_y_min = 0
        
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_fov_deg = self.get_parameter('camera_fov_deg').value

        self.roi_x_max = 512
        self.roi_y_max = 400

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )

        # Caricamento modello YOLO
        package_share_directory = get_package_share_directory('cr_vision')
        model_path = os.path.join(package_share_directory, 'data', 'color_cube.pt')
        self.model = YOLO(model_path)

        # Subscriber sincronizzati per RGB e Depth
        self.rgb_sub = Subscriber(self, Image, "/cr_vision/mirrored_camera/rgb", qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, "/cr_vision/mirrored_camera/depth", qos_profile=qos_profile)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.detect_objects)

        # Publisher (non usati nel plot ma lasciati per completezza)
        self.objects_overlay_publisher = self.create_publisher(Image, 'cr_vision/detected_objects_image', 1)
        self.obj_selected_pub_ = self.create_publisher(ObjectInfoArray, 'cr_vision/detected_objects', 10)

        # TF2 per trasformazioni camera→world
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # cv_bridge
        self.bridge = CvBridge()

        # Lista degli ID di classe (YOLO) da considerare
        self.allowed_class_ids = [
            cid for cid, name in self.model.names.items()
            if name in self.target_labels
        ]
        if not self.allowed_class_ids:
            self.get_logger().warn(
                f"Nessuna label fra {self.target_labels} è presente nel modello!"
            )

        # Salvo dimensioni correnti (h, w) della depth map
        self.img_h = None
        self.img_w = None

        self.get_logger().info("Object Detector attivo!")

    def detect_objects(self, rgb_image: Image, depth_image: Image):
        """
        1) Fa YOLO per bounding box
        2) Genera la point cloud 3D (camera→world)
        3) Trova la faccia top (Z_max) e ne calcola il baricentro world
        4) Calcola il “vero” baricentro in pixel (via TF world→camera + intrinseci)
        5) Plotta RGB, Z_world, top surface con croce rossa sul centro
        6) Disegna overlay con bounding box e ID
        """
        self.get_logger().info("Ricevuta coppia RGB+Depth per la detection")

        if rgb_image is None or depth_image is None:
            self.get_logger().warn("Immagini mancanti, salto elaborazione")
            return

        # A) Prepara overlay
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        overlay = cv_rgb.copy()

        # (1) YOLO per trovare bounding box valide
        boxes, names = self.get_detected_boxes(rgb_image)
        self.get_logger().info(f"YOLO ha trovato {len(boxes)} oggetti nella ROI")
        if not boxes:
            msg = ObjectInfoArray()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "object_detection"
            msg.objects = []
            self.obj_selected_pub_.publish(msg)
            # Pubblica overlay anche se vuoto
            img_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            img_msg.header = rgb_image.header
            self.objects_overlay_publisher.publish(img_msg)
            return

        detected_objs = []

        for i, box in enumerate(boxes):
            # Estrai la point cloud dal box
            points_map = self.compute_pointcloud_from_box(depth_image, box)
            if points_map is None or points_map.size == 0:
                continue

            # Trova centroid world e top_points
            centroid_world, centroid_pixel, top_points = self.find_top_surface_center(points_map)

            # Se ho un centro valido, registro e disegno
            if centroid_world is not None:
                xw, yw, zw = centroid_world
                self.get_logger().info(f"Centro top surface world: x={xw:.3f}, y={yw:.3f}, z={zw:.3f}")

                # prepara msg ObjectInfo
                obj = ObjectInfo()
                obj.id = i 
                cls_id = int(box.cls[0])            
                obj.label = f"{names[cls_id]}"
                obj.center.x = float(xw)
                obj.center.y = float(yw)
                obj.center.z = float(zw)
                obj.size.x  = self.target_size_x
                obj.size.y  = self.target_size_y
                obj.size.z  = self.target_size_z
                detected_objs.append(obj)

                # pubblica tf
                self.publish_tf(
                        float(xw),
                        float(yw),
                        float(zw),
                        f"{i}_top_center"
                    )

                # Estrai coordinate del box come interi
                x_min_f, y_min_f, x_max_f, y_max_f = box.xyxy[0]
                x_min, y_min, x_max, y_max = map(int, (x_min_f, y_min_f, x_max_f, y_max_f))

                # B) Disegna bounding box
                cv2.rectangle(
                    overlay,
                    (x_min, y_min),
                    (x_max, y_max),
                    (32, 32, 32),
                    2
                )

                # C) Disegna etichetta sul lato sinistro, centrata verticalmente
                label = f"{obj.label}"

                # Font piccolo e fine
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                thickness = 2

                # Calcola dimensioni del testo
                (font_w, font_h), _ = cv2.getTextSize(label, font, font_scale, thickness)

                # Posizione: lato sinistro, centrato verticalmente
                padding = 4
                text_x = x_min - font_w - padding
                text_y = y_min + (y_max - y_min) // 2 + font_h // 2  # centro verticale del box

                # Sfondo grigio scuro dietro il testo
                cv2.rectangle(
                    overlay,
                    (text_x - 2, text_y - font_h - 2),
                    (text_x + font_w + 2, text_y + 2),
                    (32, 32, 32),  # grigio scuro
                    -1
                )

                # Testo giallo sopra lo sfondo
                cv2.putText(
                    overlay,
                    label,
                    (text_x, text_y),
                    font,
                    font_scale,
                    (0, 255, 255),  # giallo
                    thickness,
                    cv2.LINE_AA
                )

            # Trasforma il centro world→camera e proietta per debug pixel
            true_cam, true_pix = self.compute_true_top_center(centroid_world)
            if true_cam and true_pix:
                Xc, Yc, Zc = true_cam
                u_c, v_c = true_pix
                self.get_logger().info(
                    f"True centro camera: ({Xc:.3f}, {Yc:.3f}, {Zc:.3f}), pixel=({u_c},{v_c})"
                )
            else:
                self.get_logger().warn("Impossibile calcolare il vero centro top")

            # Plot comparativo (puoi anche spostarlo fuori dal loop)
            # self.plot_comparison(rgb_image, points_map, top_points, true_pix)

        # Pubblica ObjectInfoArray
        if detected_objs:
            msg = ObjectInfoArray()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "object_detection"
            msg.objects = detected_objs
            self.obj_selected_pub_.publish(msg)

        # Pubblica overlay con bounding box e ID
        img_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = rgb_image.header.frame_id
        self.objects_overlay_publisher.publish(img_msg)

    def get_detected_boxes(self, rgb_image: Image):
        cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        results = self.model.predict(cv_image, classes=self.allowed_class_ids, verbose=False)

        valid_boxes = []
        if results and hasattr(results[0], "boxes") and results[0].boxes is not None:
            for box in results[0].boxes:
                x_min, y_min, x_max, y_max = box.xyxy[0]
                cx = int((x_min + x_max) / 2.0)
                cy = int((y_min + y_max) / 2.0)
                if self.roi_x_min <= cx <= self.roi_x_max and self.roi_y_min <= cy <= self.roi_y_max:
                    valid_boxes.append(box)
        return valid_boxes, results[0].names

    def compute_pointcloud_from_box(self, depth_msg: Image, box):

        fx = fy = (self.camera_width/2) / np.tan(np.deg2rad(self.camera_fov_deg/2))
        cx, cy = self.camera_width/2, self.camera_height/2  

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        h, w = depth.shape
        self.img_h, self.img_w = h, w
        if (h, w) != (512, 512):
            self.get_logger().warn(f"Depth attesa 512×512, trovata {h}×{w}")

        x_min, y_min, x_max, y_max = [int(v) for v in box.xyxy[0]]
        us = np.arange(x_min, x_max + 1)
        vs = np.arange(y_min, y_max + 1)
        u_grid, v_grid = np.meshgrid(us, vs)
        u_flat, v_flat = u_grid.ravel(), v_grid.ravel()

        Z_flat = depth[v_flat, u_flat]
        mask = (Z_flat > 0.0) & (~np.isnan(Z_flat))
        u_ok, v_ok, Z_ok = u_flat[mask], v_flat[mask], Z_flat[mask]

        # back‐projection → camera frame
        X_cam = (u_ok - cx) * Z_ok / fx
        Y_cam = (v_ok - cy) * Z_ok / fy

        # trasformo in world
        now = rclpy.time.Time()
        try:
            tf = self.tf_buffer.lookup_transform('world', 'camera_rgbd', now)
        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
            return None

        rows = []
        for i in range(len(Z_ok)):
            pt_cam = geometry_msgs.msg.PointStamped()
            pt_cam.header.frame_id, pt_cam.header.stamp = "camera_rgbd", now.to_msg()
            pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = float(X_cam[i]), float(Y_cam[i]), float(Z_ok[i])
            pt_w = tf2_geometry_msgs.do_transform_point(pt_cam, tf)
            rows.append([
                int(u_ok[i]), int(v_ok[i]),
                X_cam[i],     Y_cam[i],     Z_ok[i],
                pt_w.point.x, pt_w.point.y, pt_w.point.z
            ])

        return np.array(rows, dtype=np.float32)  # shape (N,8)

    def find_top_surface_center(self, points_map):
        # prendo Z_w che è colonna 7
        z_w = points_map[:, 7]
        z_max = np.max(z_w)
        tol = 1e-3
        mask = np.abs(z_w - z_max) < tol
        top = points_map[mask]
        if top.size == 0:
            self.get_logger().warn("Nessun top_point")
            return None, None, None
        
        num_top = top.shape[0]
        self.get_logger().info(f"[find_top_surface_center] Numero di top_points: {num_top}")

        # CENTRO CON MEDIAN invece della media (più robusto)
        X_w = np.median(top[:, 5])
        Y_w = np.median(top[:, 6])
        Z_w = np.median(top[:, 7])

        u_mean, v_mean = np.mean(top[:, 0:2], axis=0)  # opzionale: potresti farli anche median
        return (X_w, Y_w, Z_w), (int(u_mean), int(v_mean)), top

    def compute_true_top_center(self, centroid_world):
        """
        centroid_world = (X_w, Y_w, Z_w) in frame 'world'
        → trasformo in camera_rgbd e poi proietto in pixel.
        """
        if centroid_world is None:
            return None, None

        from geometry_msgs.msg import PointStamped

        # 1) Punto in world
        pt_w = PointStamped()
        pt_w.header.frame_id = 'world'
        pt_w.header.stamp = rclpy.time.Time().to_msg()

        # estraggo e casto a Python float
        Xw, Yw, Zw = centroid_world
        pt_w.point.x = float(Xw)
        pt_w.point.y = float(Yw)
        pt_w.point.z = float(Zw)

        # 2) Trasformo da world -> camera_rgbd
        try:
            t = self.tf_buffer.lookup_transform(
                'camera_rgbd',  # target_frame
                'world',        # source_frame
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f"[compute_true_top_center] TF error: {e}")
            return None, None

        pt_cam = tf2_geometry_msgs.do_transform_point(pt_w, t)
        Xc, Yc, Zc = pt_cam.point.x, pt_cam.point.y, pt_cam.point.z

        fx = fy = (self.camera_width/2) / np.tan(np.deg2rad(self.camera_fov_deg/2))
        cx, cy = self.camera_width/2, self.camera_height/2  

        if Zc <= 0.0:
            return (Xc, Yc, Zc), None

        u = int(round((Xc * fx) / Zc + cx))
        v = int(round((Yc * fy) / Zc + cy))

        # 4) Clamp su [0,511]
        u = max(0, min(511, u))
        v = max(0, min(511, v))

        return (Xc, Yc, Zc), (u, v)

    def plot_comparison(self, rgb_msg, points_map, top_points, true_pix):
        # converto RGB e preparo le mappe Z
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)

        z_img = np.zeros((self.img_h, self.img_w), dtype=np.float32)
        for u, v, *_ , z_w in points_map:
            z_img[int(v), int(u)] = z_w

        z_top = np.zeros_like(z_img)
        for u, v, *_ , z_w in top_points:
            z_top[int(v), int(u)] = z_w

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        # RGB
        axes[0].imshow(rgb); axes[0].axis('off'); axes[0].set_title("RGB")
        # Z_world
        im1 = axes[1].imshow(z_img, cmap='viridis', origin='upper', vmin=1e-6)
        axes[1].set_title("Z_world"); plt.colorbar(im1, ax=axes[1])
        # top surface
        im2 = axes[2].imshow(z_top, cmap='viridis', origin='upper', vmin=1e-6)
        axes[2].set_title("Top surface"); plt.colorbar(im2, ax=axes[2])

        # croce rossa in tutti e tre
        if true_pix is not None:
            u_c, v_c = true_pix
            for ax in axes:
                ax.scatter(u_c, v_c, s=60, c='red', marker='x')

        plt.tight_layout()
        plt.show()

    def publish_tf(self, x_world: float, y_world: float, z_world: float, object_name: str) -> None:
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  # frame padre
        t.child_frame_id = f"{object_name}_frame"

        t.transform.translation.x = x_world
        t.transform.translation.y = y_world
        t.transform.translation.z = z_world

        # Orientamento identity
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

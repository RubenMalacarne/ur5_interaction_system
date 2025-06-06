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

from cr_interfaces.msg import ObjectDetectionBox, ObjectDetectionResult, ObjectInfoArray, ObjectInfo


class ObjectDetectionNode(Node):
    """
    ROS2 node to detect object with Yolo and use depth camera to know the distance.
    there is a ROI (Region of Interest) defined by the user, to filter the objects.
    data and image result are publish in  `cr_vision/yolov8_detection_image` and `ObjectDetectionResult`.
    """

    def __init__(self) -> None:
        super().__init__('object_detection_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_labels', ["green_cube", "red_cube"]),
                ('target_size_x', 0.05),
                ('target_size_y', 0.05),
                ('target_size_z', 0.15)
            ]
        )
        self.target_labels = list(
            self.get_parameter('target_labels').value          # ['green_cube', 'red_cube', ...]
        )
        self.target_size_x = self.get_parameter('target_size_x').value
        self.target_size_y = self.get_parameter('target_size_y').value
        self.target_size_z = self.get_parameter('target_size_z').value
        # === Parametri ROI ===
        self.roi_x_min = 200
        self.roi_y_min = 0
        self.roi_x_max = 512
        self.roi_y_max = 400

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
        )

        # Carica modello YOLO
        package_share_directory = get_package_share_directory('cr_vision')
        model_path = os.path.join(package_share_directory, 'data', 'yolo_cubi.pt')
        self.model = YOLO(model_path)

        # Sottoscrizioni immagini
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

        # Publisher immagini annotate e risultati
        self.image_publisher = self.create_publisher(Image, 'cr_vision/yolov8_detection_image', 1)
        self.detection_publisher = self.create_publisher(
            ObjectDetectionResult,
            'cr_vision/yolov8_detection_results',
            1
        )
        
        # NUOVO: Publisher per oggetti rilevati con overlay RGB-Depth
        self.objects_overlay_publisher = self.create_publisher(
            Image, 
            'cr_vision/detected_objects_rgb_depth_overlay', 
            1
        )
        self.obj_selected_pub_ = self.create_publisher(
            ObjectInfoArray,
            'cr_vision/object_detection_results',
            10
        )
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Utils
        self.bridge = CvBridge()
        self.depth_image = None
        self.get_logger().info("Object Detection Node with Depth and World Transform has been started.")

        self.allowed_class_ids = [
            cid for cid, name in self.model.names.items()
            if name in self.target_labels           # ← provengono dal file YAML
        ]

        if not self.allowed_class_ids:
            self.get_logger().warn(
                f"Nessuna label fra {self.target_labels} è presente nel modello!"
            )

    # ------------------------------------------------------------------
    # CALLBACK DEPTH
    # ------------------------------------------------------------------

    def depth_callback(self, depth_data: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    # ------------------------------------------------------------------
    # FUNZIONE PER CREARE OVERLAY RGB-DEPTH
    # ------------------------------------------------------------------
    def create_rgb_depth_overlay(self, rgb_image, depth_image):
        """
        Crea un'immagine overlay combinando RGB e depth.
        La depth viene normalizzata e applicata come colormap.
        """
        # Normalizza depth per visualizzazione (0-255)
        depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
        
        # Maschera per valori validi (> 0)
        valid_depth = depth_image > 0
        if np.any(valid_depth):
            depth_min = np.min(depth_image[valid_depth])
            depth_max = np.max(depth_image[valid_depth])
            if depth_max > depth_min:
                depth_normalized[valid_depth] = ((depth_image[valid_depth] - depth_min) / 
                                                (depth_max - depth_min) * 255).astype(np.uint8)
        
        # Applica colormap alla depth (JET colormap)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        # Crea maschera per depth valida
        depth_mask = (depth_image > 0).astype(np.uint8) * 255
        depth_mask_3ch = cv2.cvtColor(depth_mask, cv2.COLOR_GRAY2BGR)
        
        # Overlay: RGB dove non c'è depth, blend dove c'è depth
        alpha = 0.6  # Trasparenza per il blend
        overlay = rgb_image.copy()
        
        # Applica depth colorata solo dove ci sono valori validi
        mask = depth_mask_3ch > 0
        overlay[mask] = (alpha * rgb_image[mask] + (1-alpha) * depth_colored[mask]).astype(np.uint8)
        
        return overlay

    # ------------------------------------------------------------------
    # FUNZIONE PER SEGMENTAZIONE DELL'OGGETTO
    # ------------------------------------------------------------------
    def segment_object_in_bbox(self, rgb_region, depth_region):
        """
        Segmenta l'oggetto dentro la bounding box usando depth e colore.
        Combina segmentazione basata su depth e colore per ottenere una maschera precisa.
        """
        h, w = rgb_region.shape[:2]
        
        # === SEGMENTAZIONE BASATA SU DEPTH ===
        depth_mask = np.zeros((h, w), dtype=np.uint8)
        
        if depth_region is not None and np.any(depth_region > 0):
            # Trova l'oggetto più vicino (depth minima valida)
            valid_depth = depth_region > 0
            if np.any(valid_depth):
                min_depth = np.min(depth_region[valid_depth])
                max_depth = np.max(depth_region[valid_depth])
                
                # Soglia adattiva: considera oggetti entro una certa distanza dal più vicino
                depth_threshold = min_depth + (max_depth - min_depth) * 0.3
                depth_mask = ((depth_region > 0) & (depth_region <= depth_threshold)).astype(np.uint8) * 255
        
        # === SEGMENTAZIONE BASATA SU COLORE (GrabCut) ===
        color_mask = np.zeros((h, w), dtype=np.uint8)
        
        if h > 10 and w > 10:  # Assicurati che la regione sia abbastanza grande
            try:
                # Inizializza maschera per GrabCut
                grabcut_mask = np.zeros((h, w), dtype=np.uint8)
                
                # Definisci un rettangolo più piccolo dentro la bbox per il foreground
                margin_x = max(1, w // 8)
                margin_y = max(1, h // 8)
                rect = (margin_x, margin_y, w - 2*margin_x, h - 2*margin_y)
                
                # Applica GrabCut
                bgd_model = np.zeros((1, 65), np.float64)
                fgd_model = np.zeros((1, 65), np.float64)
                
                cv2.grabCut(rgb_region, grabcut_mask, rect, bgd_model, fgd_model, 3, cv2.GC_INIT_WITH_RECT)
                
                # Estrai foreground
                color_mask = np.where((grabcut_mask == 2) | (grabcut_mask == 0), 0, 255).astype(np.uint8)
                
            except Exception as e:
                # Se GrabCut fallisce, usa segmentazione semplice basata sul centro
                center_x, center_y = w // 2, h // 2
                center_color = rgb_region[center_y, center_x]
                
                # Calcola differenza colore dal centro
                color_diff = np.sqrt(np.sum((rgb_region - center_color) ** 2, axis=2))
                threshold = np.std(color_diff) * 1.5
                color_mask = (color_diff < threshold).astype(np.uint8) * 255
        
        # === COMBINA LE MASCHERE ===
        if np.any(depth_mask > 0) and np.any(color_mask > 0):
            # Combina depth e colore con AND logico
            combined_mask = cv2.bitwise_and(depth_mask, color_mask)
        elif np.any(depth_mask > 0):
            # Usa solo depth se disponibile
            combined_mask = depth_mask
        elif np.any(color_mask > 0):
            # Usa solo colore se depth non disponibile
            combined_mask = color_mask
        else:
            # Fallback: usa tutta la bounding box
            combined_mask = np.ones((h, w), dtype=np.uint8) * 255
        
        # === POST-PROCESSING ===
        # Rimuovi piccoli buchi e componenti
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        # Trova la componente connessa più grande
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Prendi il contorno più grande
            largest_contour = max(contours, key=cv2.contourArea)
            combined_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(combined_mask, [largest_contour], 255)
        
        return combined_mask

        
    # ------------------------------------------------------------------
    # FUNZIONE PER TROVARE IL PUNTO CENTRALE DELLA PARTE PIÙ ALTA (WORLD)
    # ------------------------------------------------------------------
    def find_top_center_point_3d(self,
                                object_mask: np.ndarray,
                                depth_region: np.ndarray,
                                x_offset: int,
                                y_offset: int):
        """
        Trova il baricentro dei punti più alti dell'oggetto (in world),
        restituendo sia le coordinate 2-D immagine, sia le 3-D camera e world.

        • "Più alti" ora significa con coordinata Z_world massima  
        (rispetto al frame /world), non più riga-pixel minima.
        • Il TF camera→world viene calcolato una sola volta e applicato a
        tutti i punti candidati, evitando decine di lookup TF.
        """

        # --------------------------------------- controlli preliminari
        if depth_region is None or not np.any(object_mask > 0):
            return None

        # punti (u,v) mascherati nell'immagine bbox
        ys, xs = np.where(object_mask > 0)
        if ys.size == 0:
            return None

        # --------- ottieni una sola volta la matrice di trasformazione
        try:
            tf = self.tf_buffer.lookup_transform('world', 'camera_rgbd',
                                                rclpy.time.Time())
            transl = np.array([tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z])
            q = tf.transform.rotation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            rot = np.array([
                [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        # CORREZIONE: Usa gli stessi parametri intrinseci di yolo_detection
        h_full, w_full = self.depth_image.shape  # Dimensioni immagine completa
        fx = fy = 525.0
        cx_opt = w_full / 2.0  # Centro ottico dell'immagine COMPLETA
        cy_opt = h_full / 2.0

        world_pts, cam_pts, img_pts = [], [], []

        # ----------------------------------------------------- loop sui punti
        for u, v in zip(xs, ys):           # xs = colonne, ys = righe nella bbox
            z = float(depth_region[v, u])
            if z <= 0:
                continue

            # CORREZIONE: Converti coordinate bbox in coordinate immagine assolute
            u_abs = u + x_offset  # Coordinata assoluta nell'immagine
            v_abs = v + y_offset  # Coordinata assoluta nell'immagine

            # --- 3-D camera usando coordinate assolute
            x_c = (u_abs - cx_opt) * z / fx
            y_c = (v_abs - cy_opt) * z / fy
            cam = np.array([x_c, y_c, z])

            # --- 3-D world = R*cam + t
            world = rot @ cam + transl

            cam_pts.append(cam)
            world_pts.append(world)
            img_pts.append([u_abs, v_abs])  # Salva coordinate assolute

        if not world_pts:
            return None

        world_pts = np.asarray(world_pts)
        cam_pts   = np.asarray(cam_pts)
        img_pts   = np.asarray(img_pts)

        # ------------------------------------------------- selezione "top"
        z_max = world_pts[:, 2].max()
        band  = 0.005         # 2 cm sotto la quota massima
        top_msk = world_pts[:, 2] >= (z_max - band)

        # fallback se la banda restituisce 0 punti
        if not np.any(top_msk):
            top_msk = np.argmax(world_pts[:, 2])   # indice del punto più alto
            top_msk = np.array([top_msk])  # Converti in array per mantenere consistenza

        # ------------------------------------------------- centroidi
        center_world = world_pts[top_msk].mean(axis=0)
        center_cam   = cam_pts[top_msk].mean(axis=0)
        center_2d    = img_pts[top_msk].mean(axis=0)

        return {
            'center_2d': {
                'x': int(center_2d[0]),
                'y': int(center_2d[1])
            },
            'center_3d_camera': {
                'x': float(center_cam[0]),
                'y': float(center_cam[1]),
                'z': float(center_cam[2])
            },
            'center_3d_world': {
                'x': float(center_world[0]),
                'y': float(center_world[1]),
                'z': float(center_world[2])
            },
            'num_points': int(top_msk.sum())
        }

    # ------------------------------------------------------------------
    # FUNZIONE PER ESTRARRE OGGETTI RILEVATI CON SEGMENTAZIONE
    # ------------------------------------------------------------------
    def extract_detected_objects_overlay(self, rgb_image, depth_image, boxes, results):
        """
        Estrae e segmenta gli oggetti rilevati, creando un'immagine overlay solo delle parti segmentate.
        """
        if depth_image is None:
            self.get_logger().warn("Depth image not available for overlay")
            return None
            
        # Crea overlay completo RGB-Depth
        full_overlay = self.create_rgb_depth_overlay(rgb_image, depth_image)
        
        # Crea immagine nera delle stesse dimensioni
        h, w = rgb_image.shape[:2]
        objects_overlay = np.zeros((h, w, 3), dtype=np.uint8)
        
        valid_objects_found = False
        
        selected_objects = [] 
        
        for i, box in enumerate(boxes):
            x_min, y_min, x_max, y_max = box.xyxy[0]
            cls_id = int(box.cls[0])
            label = results[0].names[cls_id]
            
            # Centro della bounding box
            cx = int((x_min + x_max) / 2.0)
            cy = int((y_min + y_max) / 2.0)
            
            # Filtro ROI: salta se il centro è fuori
            if not (self.roi_x_min <= cx <= self.roi_x_max and
                    self.roi_y_min <= cy <= self.roi_y_max):
                continue
                
            valid_objects_found = True
            
            # Coordinate della bounding box (assicurati che siano dentro i limiti dell'immagine)
            x1 = max(0, int(x_min))
            y1 = max(0, int(y_min))
            x2 = min(w, int(x_max))
            y2 = min(h, int(y_max))
            
            # Estrai regioni RGB e depth della bounding box
            rgb_region = rgb_image[y1:y2, x1:x2]
            depth_region = depth_image[y1:y2, x1:x2] if depth_image is not None else None
            overlay_region = full_overlay[y1:y2, x1:x2]
            
            # Segmenta l'oggetto nella regione
            object_mask = self.segment_object_in_bbox(rgb_region, depth_region)
            
            # Applica la maschera all'overlay
            mask_3ch = cv2.cvtColor(object_mask, cv2.COLOR_GRAY2BGR) / 255.0
            segmented_overlay = (overlay_region * mask_3ch).astype(np.uint8)
            
            # Copia solo i pixel segmentati nell'immagine finale
            mask_bool = object_mask > 0
            objects_overlay[y1:y2, x1:x2][mask_bool] = segmented_overlay[mask_bool]
            
            # === TROVA IL PUNTO CENTRALE DELLA PARTE PIÙ ALTA ===
            top_center_info = self.find_top_center_point_3d(object_mask, depth_region, x1, y1)
            
            if top_center_info is not None:
                # Visualizza il punto centrale superiore
                center_2d = top_center_info['center_2d']
                center_3d_cam = top_center_info['center_3d_camera']
                center_3d_world = top_center_info['center_3d_world']
                
                # Disegna punto centrale superiore
                cv2.circle(objects_overlay, (center_2d['x'], center_2d['y']), 8, (255, 0, 255), -1)  # Magenta
                cv2.circle(objects_overlay, (center_2d['x'], center_2d['y']), 12, (255, 255, 255), 2)  # Bordo bianco
                
                # Aggiungi informazioni 3D del punto superiore
                info_text = f"Top: ({center_3d_world['x']:.2f}, {center_3d_world['y']:.2f}, {center_3d_world['z']:.2f})m"
                cv2.putText(
                    objects_overlay,
                    info_text,
                    (center_2d['x'] - 50, center_2d['y'] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 0, 255),
                    1
                )
                
                # Log delle coordinate per debugging
                self.get_logger().info(
                    f"Object {label}_{i} - Top Center Point:\n"
                    f"  2D: ({center_2d['x']}, {center_2d['y']})\n"
                    f"  3D Camera: ({center_3d_cam['x']:.3f}, {center_3d_cam['y']:.3f}, {center_3d_cam['z']:.3f})\n"
                    f"  3D World: ({center_3d_world['x']:.3f}, {center_3d_world['y']:.3f}, {center_3d_world['z']:.3f})\n"
                    f"  Points used: {top_center_info['num_points']}"
                )
                
                # Pubblica TF per il punto centrale superiore
                self.publish_tf(
                    center_3d_world['x'], 
                    center_3d_world['y'], 
                    center_3d_world['z'], 
                    f"{label}_{i}_top_center"
                )
                
                obj = ObjectInfo()
                
                obj.id = i
                obj.label = label
                obj.center.x = center_3d_world['x']
                obj.center.y = center_3d_world['y']
                obj.center.z = center_3d_world['z']
                obj.size.x = self.target_size_x
                obj.size.y = self.target_size_y
                obj.size.z = self.target_size_z
                
                selected_objects.append(obj)
                
            
            # Aggiungi contorno dell'oggetto segmentato
            contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                # Offset contour coordinates per l'immagine finale
                contour_offset = contour + [x1, y1]
                cv2.drawContours(objects_overlay, [contour_offset], -1, (0, 255, 0), 2)
            
            # Aggiungi label
            cv2.putText(
                objects_overlay, 
                f"{label}_{i}", 
                (x1, y1 - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.7, 
                (0, 255, 0), 
                2
            )
            
            # Aggiungi informazioni depth se disponibili
            if 0 <= cx < depth_image.shape[1] and 0 <= cy < depth_image.shape[0]:
                depth_val = depth_image[cy, cx]
                if depth_val > 0:
                    cv2.putText(
                        objects_overlay,
                        f"D: {depth_val:.2f}m",
                        (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                        1
                    )
        
        if selected_objects:
            selected_msg = ObjectInfoArray()
            detection_msg = ObjectDetectionResult()
            detection_msg.header = Header()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = "object_detection"
  
            selected_msg.objects = selected_objects 

            self.obj_selected_pub_.publish(selected_msg)
            
            
        return objects_overlay if valid_objects_found else None

    # ------------------------------------------------------------------
    # CALLBACK RGB
    # ------------------------------------------------------------------
    def rgb_callback(self, rgb_data: Image) -> None:
        # Converti immagine ROS -> OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")

        # Esegui YOLO con filtro per classi consentite
        results = self.model.predict(
            cv_image,
            classes=self.allowed_class_ids,
            verbose=False
        )

        # Se ci sono box
        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            annotated_frame = cv_image.copy()  # ← disegno personalizzato

            # Disegna rettangolo ROI
            cv2.rectangle(
                annotated_frame,
                (self.roi_x_min, self.roi_y_min),
                (self.roi_x_max, self.roi_y_max),
                (0, 0, 255), 2
            )

            # Prepara messaggio di risultati
            detection_msg = ObjectDetectionResult()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = "object_detection"
            detection_msg.roi_x_min = float(self.roi_x_min)
            detection_msg.roi_y_min = float(self.roi_y_min)
            detection_msg.roi_x_max = float(self.roi_x_max)
            detection_msg.roi_y_max = float(self.roi_y_max)

            for i, box in enumerate(boxes):
                x_min, y_min, x_max, y_max = box.xyxy[0]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = results[0].names[cls_id]

                # Centro della bounding box
                cx = int((x_min + x_max) / 2.0)
                cy = int((y_min + y_max) / 2.0)

                # Filtro ROI: salta se il centro è fuori
                if not (self.roi_x_min <= cx <= self.roi_x_max and
                        self.roi_y_min <= cy <= self.roi_y_max):
                    continue

                # Calcolo 3D
                x_3d, y_3d, z_3d, world_x, world_y, world_z, distance = self.yolo_detection(
                    cx, cy, x_min, y_max, annotated_frame, rgb_data
                )

                # Disegna solo le box nella ROI
                cv2.rectangle(
                    annotated_frame,
                    (int(x_min), int(y_min)),
                    (int(x_max), int(y_max)),
                    (0, 255, 0), 2
                )
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(
                    annotated_frame, label,
                    (int(x_min), int(y_min) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

                # Aggiunge box al messaggio
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

                # Pubblica TF per l'oggetto
                self.publish_tf(world_x, world_y, world_z, f"{label}_{i}")

            # NUOVO: Crea e pubblica immagine overlay degli oggetti rilevati
            objects_overlay = self.extract_detected_objects_overlay(cv_image, self.depth_image, boxes, results)
            if objects_overlay is not None:
                try:
                    overlay_msg = self.bridge.cv2_to_imgmsg(objects_overlay, "bgr8")
                    overlay_msg.header = rgb_data.header  # Mantieni l'header originale
                    self.objects_overlay_publisher.publish(overlay_msg)
                except Exception as e:
                    self.get_logger().error(f"Error publishing objects overlay: {e}")

            # Pubblica solo se hai almeno una box valida
            if detection_msg.boxes:
                self.detection_publisher.publish(detection_msg)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_publisher.publish(annotated_msg)
            else:
                self.get_logger().debug("Nessuna box valida dentro la ROI.")
        else:
            self.get_logger().debug("Nessun oggetto rilevato dal modello.")

    # ------------------------------------------------------------------
    # PROIEZIONE 3D
    # ------------------------------------------------------------------

    def yolo_detection(self, cx, cy, x_min, y_max, annotated_frame, rgb_data):
        x_3d = y_3d = z_3d = None
        world_x = world_y = world_z = 0.0
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

                    # Etichetta 3D sul frame
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

                    # Trasforma in frame mondo
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

    # ------------------------------------------------------------------
    # PUBBLICA TF OGGETTO
    # ------------------------------------------------------------------

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


# ----------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------

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
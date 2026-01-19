import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os  # <--- Ajout indispensable pour trouver le fichier

def pixel_to_spatial(u, v, depth_mm, fx, fy, cx, cy):
    z_meters = depth_mm / 1000.0
    if z_meters <= 0: return 0.0, 0.0, 0.0
    x_meters = (u - cx) * z_meters / fx
    y_meters = (v - cy) * z_meters / fy
    return x_meters, y_meters, z_meters

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.get_logger().info("IA Custom Démarrée (Mode: best.pt - TOUT détecter)...")

        self.declare_parameter('topic_rgb', '/rgb_camera/image')
        self.declare_parameter('topic_depth', '/depth_camera/image')
        self.declare_parameter('topic_info', '/camera/camera_info')

        topic_rgb = self.get_parameter('topic_rgb').value
        topic_depth = self.get_parameter('topic_depth').value
        topic_info = self.get_parameter('topic_info').value

        # --- CHARGEMENT DU MODÈLE CUSTOM ---
        # On récupère le chemin du dossier où se trouve ce script
        current_dir = os.path.dirname(os.path.realpath(__file__))
        # On construit le chemin vers best.pt
        model_path = os.path.join(current_dir, "model_v1.pt")
        
        self.get_logger().info(f"Chargement du modèle depuis : {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("Modèle best.pt chargé avec succès !")
        except Exception as e:
            self.get_logger().error(f"ERREUR chargement modèle : {e}")
            self.get_logger().error("Vérifie que best.pt est bien dans le dossier 'inf_sim/inf_sim/' !")
        
        self.bridge = CvBridge()
        self.latest_depth_img = None
        self.fx, self.fy, self.cx, self.cy = None, None, None, None

        # Pas de target_classes : on veut tout voir

        self.create_subscription(Image, topic_rgb, self.rgb_callback, 10)
        self.create_subscription(Image, topic_depth, self.depth_callback, 10)
        self.create_subscription(CameraInfo, topic_info, self.info_callback, 10)
        self.pub_coord = self.create_publisher(PointStamped, '/target/position_3d', 10)

    def info_callback(self, msg):
        if self.fx is None:
            K = msg.k
            self.fx, self.cx = K[0], K[2]
            self.fy, self.cy = K[4], K[5]

    def depth_callback(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_img = np.array(cv_depth, dtype=np.float32)
        except: pass

    def rgb_callback(self, msg):
        if self.fx is None: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # On lance la détection (conf=0.25 pour être assez permissif au début)
        results = self.model(frame, verbose=False, conf=0.25)
        
        for result in results:
            for box in result.boxes:
                # 1. Récupération des infos brutes
                cls_id = int(box.cls[0])
                name = self.model.names[cls_id] # Le nom donné lors de l'entraînement (ex: 'trash')
                
                # Coordonnées pixel
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                
                # 2. Calcul 3D
                if self.latest_depth_img is not None:
                    try:
                        h, w = self.latest_depth_img.shape
                        if 0 <= v < h and 0 <= u < w:
                            dist = self.latest_depth_img[v, u]
                            
                            if not np.isnan(dist) and not np.isinf(dist):
                                # Conversion m/mm selon ce que renvoie le simu
                                dist_mm = dist * 1000.0 if dist < 100.0 else dist
                                rx, ry, rz = pixel_to_spatial(u, v, dist_mm, self.fx, self.fy, self.cx, self.cy)
                                
                                # --- LOG IMPORTANT ---
                                # Affiche l'ID et le Nom pour que tu saches comment mapper tes objets
                                self.get_logger().info(f"DÉTECTION: ID={cls_id} ({name}) à Z={rz:.2f}m")
                                
                                # Affichage Texte
                                label = f"{name} {rz:.2f}m"
                                cv2.putText(frame, label, (x1, y1-10), 0, 1, (0,255,0), 2)
                                
                                # Publication ROS
                                p = PointStamped()
                                p.header.stamp = self.get_clock().now().to_msg()
                                p.header.frame_id = "camera_link"
                                p.point.x, p.point.y, p.point.z = rx, ry, rz
                                self.pub_coord.publish(p)
                    except: pass
                
                # Rectangle Vert pour tout le monde
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("IA Vision - Custom Model", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(InferenceNode())
    rclpy.shutdown()
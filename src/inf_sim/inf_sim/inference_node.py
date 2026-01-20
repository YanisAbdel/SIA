print("--- DÉMARRAGE DU SCRIPT ---")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os

def pixel_to_spatial(u, v, depth_mm, fx, fy, cx, cy):
    z_meters = depth_mm / 1000.0
    if z_meters <= 0: return 0.0, 0.0, 0.0
    x_meters = (u - cx) * z_meters / fx
    y_meters = (v - cy) * z_meters / fy
    return x_meters, y_meters, z_meters

class InferenceNode(Node):
    def __init__(self): 
        super().__init__('inference_node')
        self.get_logger().info("IA Class-Aware Démarrée (Envoi ID dans frame_id)...")

        # --- CORRECTION DES TOPICS ICI ---
        # On met les noms que tu as trouvés avec 'ros2 topic list'
        self.declare_parameter('topic_rgb', '/camera/image_raw')
        self.declare_parameter('topic_depth', '/camera/depth/image_raw')
        self.declare_parameter('topic_info', '/camera/camera_info')

        topic_rgb = self.get_parameter('topic_rgb').value
        topic_depth = self.get_parameter('topic_depth').value
        topic_info = self.get_parameter('topic_info').value

        current_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(current_dir, "model_v1.pt")
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"Modèle chargé : {model_path}")
        except Exception as e:
            self.get_logger().error(f"ERREUR YOLO: {e}")
        
        self.bridge = CvBridge()
        self.latest_depth_img = None
        self.fx, self.fy, self.cx, self.cy = None, None, None, None

        self.create_subscription(Image, topic_rgb, self.rgb_callback, 10)
        self.create_subscription(Image, topic_depth, self.depth_callback, 10)
        self.create_subscription(CameraInfo, topic_info, self.info_callback, 10)
        self.pub_coord = self.create_publisher(PointStamped, '/target/position_3d', 10)

    def info_callback(self, msg):
        if self.fx is None:
            K = msg.k
            self.fx, self.cx, self.fy, self.cy = K[0], K[2], K[4], K[5]

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

        detected_objects_batch = []
        # On utilise le modèle YOLO chargé
        if hasattr(self, 'model'):
            results = self.model(frame, verbose=False, conf=0.25)
            
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    name = self.model.names[cls_id] # ex: 'bicycle', 'trash'
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    if self.latest_depth_img is not None:
                        try:
                            h, w = self.latest_depth_img.shape
                            if 0 <= v < h and 0 <= u < w:
                                dist = self.latest_depth_img[v, u]
                                if not np.isnan(dist) and not np.isinf(dist):
                                    dist_mm = dist * 1000.0 if dist < 100.0 else dist
                                    rx, ry, rz = pixel_to_spatial(u, v, dist_mm, self.fx, self.fy, self.cx, self.cy)
                                    
                                    self.get_logger().info(f"DÉTECTION: {name} à {rz:.2f}m")
                                    label = f"{name} {rz:.2f}m"
                                    cv2.putText(frame, label, (x1, y1-10), 0, 1, (0,255,0), 2)
                                    
                                    p = PointStamped()
                                    p.header.stamp = msg.header.stamp 
                                    
                                    # Format spécial pour le Mapper Class-Aware
                                    p.header.frame_id = f"base_link:{name}"
                                    
                                    p.point.x = float(rz)
                                    p.point.y = float(-rx)
                                    p.point.z = float(-ry + 0.2)
                                    
                                    detected_objects_batch.append(p)
                        except Exception as e: pass
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if len(detected_objects_batch) > 0:
                for point_msg in detected_objects_batch:
                    self.pub_coord.publish(point_msg)

        cv2.imshow("IA Vision", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(InferenceNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
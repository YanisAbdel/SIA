#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import depthai as dai
import cv2
import numpy as np
import os
import sys

# --- CONFIGURATION (À MODIFIER POUR TA PI) ---
# Mets le chemin ABSOLU où tu vas poser le fichier .blob sur la Pi
# Exemple : "/home/pi/ros2_ws/src/mon_package/models/best.blob"
BLOB_PATH_HARDCODED = "/home/turtlebot2/best.blob" 

# --- FONCTION DE DÉCODAGE (Host-Side) ---
def decode_yolov8(output_tensor, conf_thres=0.5, iou_thres=0.5):
    output = np.array(output_tensor)
    if output.ndim == 3: output = output[0]
    output = output.transpose()
    
    scores = np.max(output[:, 4:], axis=1)
    mask = scores > conf_thres
    output_filtered = output[mask]
    scores_filtered = scores[mask]
    
    if len(output_filtered) == 0: return [], [], []
    
    class_ids_filtered = np.argmax(output_filtered[:, 4:], axis=1)
    
    xc = output_filtered[:, 0]
    yc = output_filtered[:, 1]
    w = output_filtered[:, 2]
    h = output_filtered[:, 3]
    
    x1 = xc - w/2
    y1 = yc - h/2
    x2 = xc + w/2
    y2 = yc + h/2
    
    boxes_filtered = np.stack((x1, y1, x2, y2), axis=1)
    indices = cv2.dnn.NMSBoxes(boxes_filtered.tolist(), scores_filtered.tolist(), conf_thres, iou_thres)
    
    final_boxes = []
    final_scores = []
    final_ids = []
    
    if len(indices) > 0:
        for i in indices.flatten():
            final_boxes.append(boxes_filtered[i])
            final_scores.append(scores_filtered[i])
            final_ids.append(class_ids_filtered[i])
            
    return final_boxes, final_scores, final_ids

class InferenceNode(Node): # J'ai gardé le nom de ta classe d'origine
    def __init__(self):
        super().__init__('inference_node')
        self.get_logger().info("🚀 Démarrage OAK-D (Mode Démo)...")

        # Vérification du fichier blob
        if os.path.exists(BLOB_PATH_HARDCODED):
            self.blob_path = BLOB_PATH_HARDCODED
        else:
            # Fallback : cherche dans le dossier courant
            current_dir = os.path.dirname(os.path.realpath(__file__))
            self.blob_path = os.path.join(current_dir, "best.blob")
            
        if not os.path.exists(self.blob_path):
            self.get_logger().error(f"❌ BLOB INTROUVABLE : {self.blob_path}")
            self.get_logger().error("Copiez 'best.blob' dans /home/pi/ ou modifiez BLOB_PATH_HARDCODED")
            # On ne quitte pas brutalement pour laisser ROS respirer, mais ça ne marchera pas
            return

        self.get_logger().info(f"✅ Modèle trouvé : {self.blob_path}")

        # --- PIPELINE DEPTHAI ---
        self.pipeline = dai.Pipeline()

        # RGB
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setPreviewSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setFps(30)

        # Stéréo
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setSubpixel(True)
        
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        # Neural Network (Mode RAW)
        nn = self.pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(self.blob_path)
        camRgb.preview.link(nn.input)

        # Sorties XLink
        xOutNn = self.pipeline.create(dai.node.XLinkOut)
        xOutNn.setStreamName("nn")
        nn.out.link(xOutNn.input)

        xOutDepth = self.pipeline.create(dai.node.XLinkOut)
        xOutDepth.setStreamName("depth")
        stereo.depth.link(xOutDepth.input)

        # Publisher ROS
        self.pub_coord = self.create_publisher(PointStamped, '/target/position_3d', 10)

        # Initialisation Device
        try:
            self.device = dai.Device(self.pipeline)
            self.qNn = self.device.getOutputQueue("nn", maxSize=4, blocking=False)
            self.qDepth = self.device.getOutputQueue("depth", maxSize=4, blocking=False)
            self.timer = self.create_timer(0.01, self.run_inference)
            self.get_logger().info("✅ Caméra connectée !")
        except Exception as e:
            self.get_logger().error(f"❌ Impossible de connecter la OAK-D : {e}")

        self.labels = ["objet"] * 80 

    def run_inference(self):
        if not hasattr(self, 'device'): return

        inNn = self.qNn.tryGet()
        inDepth = self.qDepth.tryGet()

        if inNn and inDepth:
            # 1. Récupération des données brutes
            layers = inNn.getAllLayerNames()
            output_tensor = inNn.getLayerFp16(layers[0])
            
            # Reshape (84 lignes, 8400 colonnes)
            try:
                output_tensor = np.array(output_tensor).reshape(1, 84, 8400)
            except: return

            # 2. Décodage local (Python)
            boxes, scores, ids = decode_yolov8(output_tensor)
            
            # 3. Récupération Depth
            depthFrame = inDepth.getFrame() # mm

            for box, score, cls_id in zip(boxes, scores, ids):
                x1, y1, x2, y2 = map(int, box)
                
                # Calcul de la profondeur au centre de la bounding box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Sécurité dimensions
                if 0 <= cx < 640 and 0 <= cy < 640:
                    z_mm = depthFrame[cy, cx]
                    
                    if z_mm > 0: # Si la profondeur est valide
                        z_meters = z_mm / 1000.0
                        
                        # --- TRANSFORMATION ROBOT ---
                        x_robot = z_meters
                        y_robot = -((cx - 320) * z_meters / 500.0) # Approx FOV
                        z_robot = 0.0
                        
                        # Publication
                        msg = PointStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "base_link:objet"
                        
                        msg.point.x = float(x_robot)
                        msg.point.y = float(y_robot)
                        msg.point.z = float(z_robot)
                        
                        self.pub_coord.publish(msg)
                        self.get_logger().info(f"🎯 Détection : X={x_robot:.2f}m, Y={y_robot:.2f}m")

def main():
    rclpy.init()
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
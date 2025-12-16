import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class OakListener(Node):
    def __init__(self):
        super().__init__('oak_listener')
        self.br = CvBridge()

        # CONFIGURATION QoS: On s'aligne EXACTEMENT sur le Publisher (RELIABLE)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Match avec la caméra
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Abonnement RGB
        self.sub_rgb = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.rgb_callback,
            qos_reliable 
        )

        # 2. Abonnement Profondeur
        self.sub_depth = self.create_subscription(
            Image,
            '/oak/stereo/image_raw',
            self.depth_callback,
            qos_reliable
        )

        self.get_logger().info("OAK-D Listener démarré en mode RELIABLE. En attente...")

    def rgb_callback(self, msg):
        # LOG DE DEBUG : On prouve que le message arrive !
        self.get_logger().info(f"Reçu Image RGB ! Taille: {msg.width}x{msg.height}")
        
        try:
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Flux RGB", current_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Erreur affichage RGB: {e}")

    def depth_callback(self, msg):
        # On ne log pas ici pour ne pas spammer, juste le RGB suffit pour tester
        try:
            depth_frame = self.br.imgmsg_to_cv2(msg, "16UC1")
            # Normalisation pour affichage
            depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.uint8(depth_normalized)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            cv2.imshow("Flux Profondeur", depth_colored)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Erreur affichage Depth: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OakListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

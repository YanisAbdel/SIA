import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped # Import du type de message
from std_msgs.msg import Header # Pour l'en-tête du message

# Importations de Luxonis/DepthAI (selon votre script actuel)
import depthai as dai 


class OakdObjectPublisher(Node):
    def __init__(self):
        super().__init__('oakd_object_publisher')
        
        # 1. Créer le publisher
        self.publisher_ = self.create_publisher(PointStamped, '/oakd_object_position_cam', 10)
        
        # 2. Initialiser la caméra et la pipeline DepthAI ici (logique existante)
        # self.pipeline = dai.Pipeline()
        # ... 

        # 3. Utiliser un timer ou un thread dédié pour lire et publier
        self.timer = self.create_timer(0.1, self.publish_object_position) # Exécution toutes les 100 ms

    def publish_object_position(self):
        # 1. Logique pour lire la position de l'objet (remplacez par votre code Luxonis)
        # Supposons que votre code renvoie (x, y, z)
        # x_cam, y_cam, z_cam = self.get_depthai_object_coords() 
        x_cam, y_cam, z_cam = 0.5, 0.0, 1.0 # Exemple statique pour le test

        # 2. Construire l'en-tête
        header = Header()
        header.stamp = self.get_clock().now().to_msg() # Horodatage actuel
        header.frame_id = "camera_frame" # IMPORTANT : C'est le référentiel de la caméra

        # 3. Construire le message PointStamped
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = float(x_cam)
        point_msg.point.y = float(y_cam)
        point_msg.point.z = float(z_cam)
        
        # 4. Publier
        self.publisher_.publish(point_msg)
        # self.get_logger().info(f'Published: ({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}) in camera_frame')

def main(args=None):
    rclpy.init(args=args)
    object_publisher_node = OakdObjectPublisher()
    rclpy.spin(object_publisher_node)
    object_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

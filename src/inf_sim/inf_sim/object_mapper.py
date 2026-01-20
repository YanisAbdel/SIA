import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math

class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')
        
        self.create_subscription(PointStamped, '/target/position_3d', self.target_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.objects = []
        
        # --- SEUIL DE FUSION ---
        # On peut mettre un TRES GRAND seuil maintenant (2.5m) !
        # Puisque le code vérifie le nom de l'objet, le vélo ne mangera pas la poubelle.
        # Cela permet au vélo d'être détecté comme UN SEUL objet (roue avant + arrière).
        self.threshold_dist = 2.5 
        
        self.lock_count = 30 
        self.max_detection_range = 5.0 

        self.get_logger().info("Mapper Class-Aware Prêt")

    def target_callback(self, msg):
        if msg.point.z > self.max_detection_range:
            return

        # 1. Extraction du Nom de la classe depuis le frame_id (ex: "base_link:bicycle")
        full_frame_id = msg.header.frame_id
        if ':' in full_frame_id:
            real_frame_id, label = full_frame_id.split(':')
        else:
            real_frame_id, label = full_frame_id, "unknown"

        try:
            time_zero = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',
                real_frame_id, # On utilise 'base_link' pour la mathématique
                time_zero,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            point_map = do_transform_point(msg, transform)
            x_new = point_map.point.x
            y_new = point_map.point.y
            
            found = False
            for i, obj in enumerate(self.objects):
                # --- RÈGLE D'OR : ON NE FUSIONNE QUE SI C'EST LE MÊME TYPE D'OBJET ---
                if obj['label'] != label:
                    continue # On passe au suivant si c'est pas le même type (ex: Trash vs Bicycle)

                dist = math.sqrt((obj['x'] - x_new)**2 + (obj['y'] - y_new)**2)
                
                if dist < self.threshold_dist:
                    # Fusion autorisée (Même classe + Distance OK)
                    obj['count'] += 1
                    if obj['count'] < self.lock_count:
                        n = obj['count']
                        obj['x'] = (obj['x'] * (n-1) + x_new) / n
                        obj['y'] = (obj['y'] * (n-1) + y_new) / n
                    found = True
                    break
            
            if not found:
                # Création d'un nouvel objet avec son étiquette (label)
                self.objects.append({
                    'x': x_new, 'y': y_new, 'z': 0.0, 
                    'count': 1, 'label': label
                })
                self.get_logger().info(f"NOUVEAU {label.upper()} : X={x_new:.2f}, Y={y_new:.2f}")

            self.publish_markers()

        except Exception as e:
            self.get_logger().error(f"ERREUR: {e}")

    def publish_markers(self):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, obj in enumerate(self.objects):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detected_objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obj['x']
            marker.pose.position.y = obj['y']
            marker.pose.position.z = 0.0
            
            # --- COULEURS PAR CLASSE ---
            marker.color.a = 1.0
            if 'bicycle' in obj['label']:
                # VERT pour le vélo
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
            elif 'trash' in obj['label']:
                # ROUGE pour la poubelle
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
            else:
                # BLEU pour le reste
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0
            
            # Taille
            marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3

            # Texte
            text = Marker()
            text.header.frame_id = "map"
            text.type = Marker.TEXT_VIEW_FACING
            text.id = i + 1000
            # On affiche le nom de l'objet au dessus
            text.text = f"{obj['label']} ({i})"
            text.pose.position.x = obj['x']
            text.pose.position.y = obj['y']
            text.pose.position.z = 0.6
            text.scale.z = 0.2
            text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0

            marker_array.markers.append(marker)
            marker_array.markers.append(text)
        
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    rclpy.spin(ObjectMapper())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
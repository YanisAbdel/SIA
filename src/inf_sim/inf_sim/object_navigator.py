import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import json
import math

class ObjectNavigator(Node):
    def __init__(self):
        super().__init__('object_navigator')
        self.get_logger().info("Navigateur d'objets démarré...")
        
        # Action client pour Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriber pour les markers
        self.create_subscription(MarkerArray, '/visualization_marker_array', 
                                self.marker_callback, 10)
        
        # Charger les objets depuis le JSON
        self.json_file = "/home/saif/ma_carte_semantique.json"
        self.objects = []
        self.load_objects()
        
    def load_objects(self):
        """Charge les objets détectés depuis le fichier JSON"""
        try:
            with open(self.json_file, 'r') as f:
                self.objects = json.load(f)
                self.get_logger().info(f"Chargé {len(self.objects)} objets")
        except:
            self.get_logger().warn("Fichier JSON introuvable")
    
    def marker_callback(self, msg):
        """Met à jour la liste des objets détectés"""
        self.load_objects()
    
    def navigate_to_object(self, object_name):
        """Navigue vers un objet spécifique"""
        # Chercher l'objet dans la liste
        target = None
        for obj in self.objects:
            if obj['name'] == object_name:
                target = obj
                break
        
        if target is None:
            self.get_logger().error(f"Objet '{object_name}' non trouvé!")
            return False
        
        self.get_logger().info(f"Navigation vers {object_name} à ({target['x']}, {target['y']})")
        
        # Créer le goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position de l'objet
        goal_msg.pose.pose.position.x = float(target['x'])
        goal_msg.pose.pose.position.y = float(target['y'])
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (face à l'objet)
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Attendre que le serveur d'action soit prêt
        self._action_client.wait_for_server()
        
        # Envoyer le goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def feedback_callback(self, feedback_msg):
        """Affiche le feedback de navigation"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance restante: {feedback.distance_remaining:.2f}m")
    
    def goal_response_callback(self, future):
        """Gère la réponse du serveur d'action"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeté!')
            return
        
        self.get_logger().info('Goal accepté!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Gère le résultat final de la navigation"""
        result = future.result().result
        self.get_logger().info('Navigation terminée!')

def main():
    rclpy.init()
    navigator = ObjectNavigator()
    
    # Exemple : navigue vers le premier objet "trash" détecté
    navigator.navigate_to_object('trash')
    
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

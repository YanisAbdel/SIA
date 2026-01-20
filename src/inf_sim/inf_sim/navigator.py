import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener
import sys
import threading
import time
import math

# Fonction utilitaire pour convertir un angle (yaw) en Quaternion (ROS)
def euler_to_quaternion(yaw):
    q = PoseStamped().pose.orientation
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class ObjectNavigator(Node):
    def __init__(self):
        super().__init__('object_navigator')
        self.known_objects = {}
        
        # 1. Écoute des objets (Mapper)
        self.create_subscription(MarkerArray, '/object_markers', self.marker_callback, 10)
        
        # 2. Écoute de la position du Robot (TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def marker_callback(self, msg):
        for marker in msg.markers:
            if marker.type == Marker.TEXT_VIEW_FACING:
                self.known_objects[marker.text] = (marker.pose.position.x, marker.pose.position.y)

    def get_robot_pose(self):
        try:
            # On demande où est 'base_link' par rapport à 'map'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, rclpy.duration.Duration(seconds=1.0))
            return trans.transform.translation.x, trans.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Impossible de localiser le robot: {e}")
            return None, None

def main():
    rclpy.init()

    # --- CONFIGURATION ---
    STOP_DISTANCE = 0.6  # Le robot s'arrêtera à 60cm de l'objet (pour ne pas taper dedans)

    # 1. Lancement du Node d'écoute (Background)
    listener_node = ObjectNavigator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(listener_node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    print("Navigateur prêt ! En attente de données...")
    time.sleep(2.0) 

    # 2. Nav2 (Le Chef)
    navigator = BasicNavigator()

    # 3. Boucle Principale
    try:
        while rclpy.ok():
            objects = listener_node.known_objects.copy()
            robot_x, robot_y = listener_node.get_robot_pose()

            print("\n" + "="*50)
            print(f"--- {len(objects)} OBJETS DÉTECTÉS ---")
            
            if robot_x is None:
                print(" ATTENTE LOCALISATION ROBOT (TF)...")
            elif not objects:
                print("AUCUN OBJET... (Vérifiez le Mapper)")
            else:
                for name in sorted(objects.keys()):
                    pos = objects[name]
                    # Calcul de la distance à vol d'oiseau pour info
                    dist_to_obj = math.sqrt((pos[0]-robot_x)**2 + (pos[1]-robot_y)**2)
                    print(f" 📍 {name:<15} : [x={pos[0]:.2f}, y={pos[1]:.2f}] (à {dist_to_obj:.2f}m)")
            
            print("="*50)
            target_name = input("Cible (ou 'q') > ").strip()

            if target_name == 'q':
                break
            if target_name == "":
                continue

            if target_name in objects:
                obj_x, obj_y = objects[target_name]
                
                # --- CALCUL INTELLIGENT DU POINT D'ARRIVÉE ---
                if robot_x is not None:
                    # 1. Vecteur Robot -> Objet
                    dx = obj_x - robot_x
                    dy = obj_y - robot_y
                    dist_total = math.sqrt(dx**2 + dy**2)
                    
                    # 2. On recule le point d'arrivée de STOP_DISTANCE
                    if dist_total > STOP_DISTANCE:
                        ratio = (dist_total - STOP_DISTANCE) / dist_total
                        target_x = robot_x + dx * ratio
                        target_y = robot_y + dy * ratio
                    else:
                        print("Vous êtes déjà trop près ! Je reste sur place.")
                        continue

                    # 3. Calcul de l'angle pour regarder l'objet
                    yaw = math.atan2(dy, dx)
                    
                    print(f"\n--> CALCUL TRAJECTOIRE :")
                    print(f"    Objet réel : ({obj_x:.2f}, {obj_y:.2f})")
                    print(f"    Point d'arrêt (Safe) : ({target_x:.2f}, {target_y:.2f})")
                    
                    # Création de la pose
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                    goal_pose.pose.position.x = target_x
                    goal_pose.pose.position.y = target_y
                    goal_pose.pose.orientation = euler_to_quaternion(yaw) # Orientation correcte

                    navigator.goToPose(goal_pose)

                    # Suivi
                    i = 0
                    while not navigator.isTaskComplete():
                        i += 1
                        feedback = navigator.getFeedback()
                        if feedback and i % 5 == 0:
                            print(f"   >>> Reste : {feedback.distance_remaining:.2f}m")
                        time.sleep(0.2)

                    result = navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        print("\nARRIVÉ (DEVANT L'OBJET) !")
                    else:
                        print(f"\nRésultat : {result}")

                else:
                    print("Erreur : Je ne connais pas ma propre position (TF manquant).")
                
            else:
                print(f"\n'{target_name}' introuvable.")

    except KeyboardInterrupt:
        print("\nArrêt.")

    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
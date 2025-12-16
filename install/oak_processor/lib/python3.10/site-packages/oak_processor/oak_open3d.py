import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # Profil QoS pour les capteurs
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 # Librairie pour décoder PointCloud2
import numpy as np
import sys
import os

# Ce code n'ouvre PAS de fenêtre 3D (mode Headless)
# Il publie simplement les coordonnées X, Y, Z dans le terminal pour prouver la réception des données.

class OakOpen3D(Node):
    def __init__(self):
        super().__init__('oak_open3d_visualizer')
        
        # Variable pour stocker le dernier nuage de points reçu
        self.latest_points = None 

        # --- ABONNEMENT POINTCLOUD ---
        # Utilisation de qos_profile_sensor_data pour s'adapter au mode Best Effort du driver
        self.subscription = self.create_subscription(
            PointCloud2,
            '/oak/points',
            self.listener_callback,
            qos_profile_sensor_data 
        )
        self.get_logger().info("Nœud Open3D démarré. En attente de points...")

    def listener_callback(self, msg):
        """Fonction appelée à chaque réception d'un nuage de points."""
        
        try:
            # Lire les points (X, Y, Z) depuis le message ROS
            # Utilisation de skip_nans=True pour ignorer les points invalides (fond, ciel)
            points_generator = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            
            # Convertir le générateur en tableau Numpy
            points_list = list(points_generator)
            
            if points_list:
                # Stocker dans la variable de classe, qui sera lue par la boucle 'main'
                self.latest_points = np.array(points_list)
            
        except Exception as e:
            self.get_logger().error(f"Erreur de conversion/lecture du PointCloud: {e}")

def main(args=None):
    # Ce mode ne gère pas de fenêtre graphique (Headless)
    rclpy.init(args=args)
    node = OakOpen3D()

    print("Démarrage en mode écoute seule (Traitement des données sans fenêtre 3D)...")
    
    try:
        while rclpy.ok():
            # Nécessaire pour recevoir les données
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Si nous avons reçu de nouvelles données dans le callback
            if node.latest_points is not None:
                nb_points = len(node.latest_points)
                
                if nb_points > 0:
                    # Exemple de point de donnée (au milieu du tableau)
                    pt = node.latest_points[int(nb_points/2)] 
                    
                    # LOG DE SUCCÈS : Affiche la preuve que les données sont reçues et traitées
                    node.get_logger().info(
                        f"Reçu {nb_points} points 3D. Coordonnées centrales (X,Y,Z): "
                        f"[{pt[0]:.2f}m, {pt[1]:.2f}m, {pt[2]:.2f}m]"
                    )
                
                # Réinitialiser la variable après lecture
                node.latest_points = None

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

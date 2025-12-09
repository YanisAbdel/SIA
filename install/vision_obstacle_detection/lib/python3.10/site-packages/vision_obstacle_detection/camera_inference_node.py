import rclpy
from rclpy.node import Node




class CameraInferenceNode(Node):

    def __init__(self):

        super().__init__('camera_inference_node') 
        
        self.get_logger().info('Nœud CameraInferenceNode initialisé.')
        


def main(args=None):
    rclpy.init(args=args)

    node = CameraInferenceNode()
    
    rclpy.spin(node) 
    
    # Nettoyage
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

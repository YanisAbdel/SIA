import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    # 1. Initialisation
    rclpy.init()
    navigator = BasicNavigator()

    # 2. ATTENTE CRITIQUE (C'est ce qui manquait !)
    # On attend que Nav2 soit totalement actif avant d'envoyer quoi que ce soit.
    print("Orchestrator: Waiting for Nav2 to be fully active...")
    navigator.waitUntilNav2Active()
    print("Orchestrator: Nav2 is ready! Starting patrol.")

    # 3. Définition des points (Coordonnées sûres que tu as testées)
    goal_poses = []
    
    # Goal 1
    goal1 = PoseStamped()
    goal1.header.frame_id = 'map'
    goal1.pose.position.x = 0.5  # Adapte ces valeurs selon ta carte !
    goal1.pose.position.y = 0.0
    goal1.pose.orientation.w = 1.0
    goal_poses.append(goal1)

    # Goal 2
    goal2 = PoseStamped()
    goal2.header.frame_id = 'map'
    goal2.pose.position.x = 0.0
    goal2.pose.position.y = 0.0
    goal2.pose.orientation.w = 1.0
    goal_poses.append(goal2)

    # 4. Boucle d'exécution
    for i, goal in enumerate(goal_poses):
        print(f'Orchestrator: Going to goal {i+1}...')
        navigator.goToPose(goal)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # On imprime un petit point pour montrer que ça vit
            # print('.', end='', flush=True) 
            time.sleep(0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'\nOrchestrator: Goal {i+1} reached!')
            time.sleep(2.0) # Pause à chaque point
        elif result == TaskResult.CANCELED:
            print(f'\nOrchestrator: Goal {i+1} was canceled!')
        elif result == TaskResult.FAILED:
            print(f'\nOrchestrator: Goal {i+1} failed!')

    # 5. Fin
    print("Orchestrator: Patrol finished.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

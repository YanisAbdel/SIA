# SIA

Ce projet implémente une solution de navigation sémantique sur un robot TurtleBot3 simulé dans Gazebo (environnement domestique). Le système utilise une caméra OAK-D simulée pour détecter des objets spécifiques (poubelles, vélos, etc.) via YOLOv8, les positionner en 3D sur une carte, et permettre à l'utilisateur de commander le robot pour qu'il se rende automatiquement devant un objet choisi.

## Architecture du Projet

### Le système est divisé en 4 nœuds principaux communiquant via ROS 2 :

#### Simulation & Hardware (Gazebo) :

Simule le TurtleBot3 Burger équipé d'une caméra OAK-D-Pro.

Gère le pont (Bridge) ROS 2 <-> Gazebo et les transformations TF.

#### Perception (Inference Node) :

Récupère les images RGB et de Profondeur.

Utilise YOLOv8 pour la détection d'objets (trash, bicycle, etc.).

Projette les pixels détectés en coordonnées 3D spatiales (X, Y, Z) par rapport au robot.

#### Cartographie Sémantique (Object Mapper) :

Écoute les positions 3D brutes.

Transforme les coordonnées vers le repère global (map).

Filtre et fusionne les détections (moyenne glissante) pour stabiliser la position des objets.

Publie des Markers visuels dans RViz.

#### Orchestrateur (Navigator) :

Interface utilisateur (CLI).

Liste les objets détectés en temps réel.

Envoie les commandes de navigation à Nav2 pour se déplacer intelligemment vers l'objet cible (arrêt de sécurité à 60cm).

### Prérequis et Installation
1. Environnement
Ubuntu 22.04 LTS

ROS 2 Humble Hawksbill

Python 3.10+

2. Dépendances ROS 2
Assurez-vous que les paquets standards sont installés :

```bash

sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-tf2-ros

```
3. Dépendances Python
Installez les librairies nécessaires pour YOLO et la vision :
```Bash

pip3 install ultralytics opencv-python numpy

```
4. Nettoyage (Optionnel mais recommandé)

```Bash
rm -rf build/ install/ log/
```
5. Installation du Workspace
```Bash

mkdir -p ~/turtlebot3_ws
cd ~/turtlebot3_ws
# Clonez ce dépôt ici
git clone https://github.com/YanisAbdel/SIA .
cd /src
colcon build --symlink-install
source install/setup.bash
```
### Guide de Démarrage (Exécution)
Pour lancer la démonstration complète, vous aurez besoin de 6 terminaux.

IMPORTANT : Avant de lancer les commandes, assurez-vous d'avoir sourcé votre environnement dans chaque terminal : source ~/turtlebot3_ws/install/setup.bash export TURTLEBOT3_MODEL=burger

Terminal 1 : Simulation (Gazebo)
Lance le monde, le robot, et le pont ROS-Gazebo.

```Bash

ros2 launch turtlebot3_gazebo start_oakd_world.launch.py
```
### Note importante pour la première installation : Il est normal que la simulation ne charge pas correctement les modèles (caméra OAK-D, objets) lors du tout premier lancement.

Le premier démarrage sert à créer automatiquement le dossier de configuration caché ~/.ignition sur votre machine. Une fois ce dossier créé (même si la simulation a échoué ou manque d'objets), coupez la simulation (Ctrl+C) et copiez le dossier fuel du projet vers ce répertoire :
```Bash

cp -r fuel/ ~/.ignition/fuel
```
relancer la commande :
```Bash

ros2 launch turtlebot3_gazebo start_oakd_world.launch.py
```
Attendez que Gazebo s'ouvre et que le robot apparaisse.

Terminal 2 : SLAM (Cartographer)
Lance l'algorithme de localisation et de cartographie (nécessaire pour créer le repère map).

```Bash

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True use_rviz:=False
```
Terminal 3 : Détection d'Objets (YOLO)

Lance le nœud d'inférence qui analyse les images de la caméra.

```Bash

python3 inf_sim/inf_sim/inference_node.py --ros-args -p use_sim_time:=true
```
Une fenêtre "IA Vision" doit s'ouvrir montrant ce que voit le robot.

Terminal 4 : Mapping Sémantique
Lance le nœud qui place les markers (sphères colorées) sur la carte.

```Bash

python3 inf_sim/inf_sim/object_mapper.py --ros-args -p use_sim_time:=true
```
Terminal 5 : Navigation Stack (Nav2)
Active le système de navigation autonome.

Note : Modifiez le chemin vers la carte .yaml si nécessaire, ou utilisez la carte générée en temps réel par Cartographer si vous n'avez pas de fichier pré-enregistré (dans ce cas, ignorez l'argument map:=... ou pointez vers votre propre fichier).

# Exemple avec une carte pré-existante (recommandé pour la navigation précise) :
```Bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/my_oakd_map.yaml use_rviz:=True
```
Si vous n'avez pas de carte my_oakd_map.yaml, le robot naviguera en mode "inconnu" ou vous devrez faire une phase de mapping d'abord.

Terminal 6 : Le Chef d'Orchestre (Navigator)
Lance l'interface de commande. C'est ici que vous contrôlez le robot.

```Bash

python3 inf_sim/inf_sim/navigator.py --ros-args -p use_sim_time:=true
```
### Utilisation

Tourner le robot (via un teleop ou en lui donnant des Nav2 Goal) pour qu'il explore la maison.

Dès que la caméra voit un objet (Trash, Bicycle...), il apparaîtra dans la liste du Terminal 6 :

```Plaintext
--- 2 OBJETS DÉTECTÉS ---
 📍 trash           : [x=2.50, y=1.10] (à 3.20m)
 📍 bicycle         : [x=-1.20, y=0.50] (à 1.50m)
Tapez le nom de l'objet dans le terminal (ex: trash) et appuyez sur Entrée.
```
Le robot calculera une trajectoire sûre et se rendra automatiquement devant l'objet.

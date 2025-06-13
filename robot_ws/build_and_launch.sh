#!/bin/bash

# Définition du chemin du dossier du workspace
WORKSPACE_DIR="/home/equipe104/INF3995-104/robot_ws"

# Se déplacer dans le dossier du workspace
cd "$WORKSPACE_DIR" || { echo "Erreur : Impossible de se déplacer dans $WORKSPACE_DIR"; exit 1; }

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash

sudo chmod 666 /dev/ttyTHS1

ros2 launch limo_bringup limo_start.launch.py

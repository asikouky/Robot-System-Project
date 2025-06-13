#!/bin/bash

# === Workspace simulation ===
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # racine du dossier robot

# Aller dans le workspace
cd "$WORKSPACE_DIR" || { echo "[ERREUR] Impossible d'accéder à $WORKSPACE_DIR"; exit 1; }

echo "[INFO] Workspace : $WORKSPACE_DIR"

# Charger ROS 2
source /opt/ros/humble/setup.bash

# Construire le workspace
colcon build --cmake-args -DBUILD_TESTING=OFF

# Sourcer le workspace
source install/setup.bash

# Lancer Gazebo (Ignition)
ros2 launch ros_gz_example_bringup diff_drive.launch.py

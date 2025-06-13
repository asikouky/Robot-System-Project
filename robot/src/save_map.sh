#!/bin/bash
source /opt/ros/humble/setup.bash

# Lancer le map_saver en background
ros2 run nav2_map_server map_saver_cli -f merged_map --ros-args -r map:=/merged_map &
MAP_SAVER_PID=$!

sleep 2

# Tuer le processus au cas où il traîne
kill $MAP_SAVER_PID 2>/dev/null

sleep 2
# Confirmer la fin
echo "✅ Carte sauvegardée et processus map_saver terminé."


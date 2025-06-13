#!/bin/bash

ENABLE_ACKERMAN=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --enable-ackerman=*)
      ENABLE_ACKERMAN="${1#*=}"
      shift
      ;;
    --enable-ackerman)
      ENABLE_ACKERMAN=true
      shift
      ;;
    *)
      shift
      ;;
  esac
done

if [ "$ENABLE_ACKERMAN" = "true" ]; then
  DRIVE_LAUNCH_FILE="ackerman.launch.py"
else
  DRIVE_LAUNCH_FILE="diff_drive.launch.py"
fi

mkdir -p ~/.vnc
echo "$VNC_PASSWORD" | /opt/TurboVNC/bin/vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd
TVNC_WM=xfce /opt/TurboVNC/bin/vncserver -geometry 1920x1080
export DISPLAY=:1
cd /workspace/robot
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch ros_gz_example_bringup $DRIVE_LAUNCH_FILE 
/bin/bash
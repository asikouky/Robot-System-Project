#!/bin/bash
IMAGE_NAME="robot-sim"
VNC_PASSWORD="abc123"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

REBUILD=false
ACKERMAN_ARG=""

if [[ "$1" == "--rebuild" ]]; then
  REBUILD=true
  shift
fi

if [[ "$1" == "--enable-ackerman" ]]; then
  ACKERMAN_ARG="--enable-ackerman"
  echo "Ackerman steering enabled"
fi

if [[ "$REBUILD" == "true" || "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
  echo "Building Docker image..."
  docker build -t $IMAGE_NAME $SCRIPT_DIR
else
  echo "Using existing Docker image. Use --rebuild flag to force a rebuild."
fi

echo "Launching container with TurboVNC..."
docker run --rm -p 5901:5901 -p 8765:8765 \
           -v $SCRIPT_DIR/src:/workspace/robot/src \
           -e VNC_PASSWORD=$VNC_PASSWORD \
           -it $IMAGE_NAME /root/startup.sh $ACKERMAN_ARG

echo "Container exited. The --rm flag was used, so the container has been removed."
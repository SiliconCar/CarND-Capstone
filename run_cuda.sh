#! /bin/bash
set -e

# Settings from environment
UDACITY_SOURCE=${UDACITY_SOURCE:-`pwd`}
UDACITY_IMAGE=${UDACITY_IMAGE:-tantony/carnd-capstone-cuda}
CONTAINER_NAME="udacity_carnd"

if [ "$(docker ps -a | grep ${CONTAINER_NAME})" ]; then
  echo "Attaching to running container..."
  nvidia-docker exec -eDISPLAY=$DISPLAY -it ${CONTAINER_NAME} bash $@
else
  nvidia-docker run --name ${CONTAINER_NAME} --rm -it -p 4567:4567 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v "${UDACITY_SOURCE}:/udacity" ${UDACITY_IMAGE} $@
fi

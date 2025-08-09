#!/bin/bash

# --- Configuration (ADJUST THESE) ---
ROS_DOMAIN_ID="0"
CONTAINER_NAME="ros_container_1"
DOCKER_IMAGE="osrf/ros:humble-desktop"
# Point to the eth0-specific profile
FAST_DDS_PROFILE_PATH="/home/kevin/ws/src/fastdds_profiles/fastdds_eth0.xml"
WORKSPACE_HOST_PATH="/home/kevin/ws/"
ROS_DISTRO="humble"


docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

sudo docker run -P -it --rm \
  --network=host \
  --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/kevin/ws/src/DDS_Profiles/fastdds_eth0.xml:/fastdds_eth0.xml:ro \
  -v "${WORKSPACE_HOST_PATH}:/ws:rw" \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  --name ${CONTAINER_NAME} \
  ${DOCKER_IMAGE} \
  bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /ws/install/setup.bash && \
    exec bash \
  "

echo "Container ${CONTAINER_NAME} exited."
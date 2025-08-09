#!/bin/bash

# --- Configuration (ADJUST THESE) ---
ROS_DOMAIN_ID="0"
CONTAINER_NAME="ros_container_2"
DOCKER_IMAGE="osrf/ros:humble-desktop"
# Point to the eth1
FAST_DDS_PROFILE_PATH="/home/kevin/ws/src/fastdds_profiles/fastdds_eth1.xml"
WORKSPACE_HOST_PATH="/home/kevin/ws/"
ROS_DISTRO="humble"

docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

sudo docker run -it --rm \
  --network=host \
  --name ${CONTAINER_NAME} \
  --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/kevin/ws/src/DDS_Profiles/fastdds_eth1.xml:/fastdds_eth1.xml:ro \
  -v "${WORKSPACE_HOST_PATH}:/ros_ws:rw" \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_eth1.xml \
  --name ${CONTAINER_NAME} \
  ${DOCKER_IMAGE} \
  bash -c " \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        source /ros_ws/install/setup.bash && \
        exec bash \
      "

echo "Container ${CONTAINER_NAME} exited."
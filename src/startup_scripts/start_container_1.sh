#!/bin/bash

# --- Configuration (ADJUST THESE) ---
ROS_DOMAIN_ID="0"
CONTAINER_NAME="ros_container_1"
DOCKER_IMAGE="osrf/ros:humble-desktop"
# Point to the eth0-specific profile
FAST_DDS_PROFILE_PATH="/home/kevin/ws/src/fastdds_profiles/fastdds_eth0.xml"
WORKSPACE_HOST_PATH="/home/kevin/ws/"
ROS_DISTRO="humble"

echo "--- Starting Docker Container 1 (Node A/B) in Host Network Mode ---"
echo "Container Name: ${CONTAINER_NAME}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "Using FastDDS Profile: ${FAST_DDS_PROFILE_PATH}" # New for clarity

docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

sudo docker run -it --rm \
  --network=host \
  --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/kevin/ws/src/DDS_Profiles/fastdds_eth0.xml:/fastdds_eth0.xml:ro \
  -v "${WORKSPACE_HOST_PATH}:/ros_ws:rw" \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_eth0.xml \
  --name ${CONTAINER_NAME} \
  ${DOCKER_IMAGE} \
  bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /ros_ws/install/setup.bash && \
    echo 'ROS2 environment loaded. You are in ${CONTAINER_NAME}.' && \
    echo 'Container IPs (same as host): $(hostname -I)' && \
    echo 'To run Node A: ros2 run NODE_A_PKG NODE_A_EXEC' && \
    exec bash \
  "

echo "Container ${CONTAINER_NAME} exited."
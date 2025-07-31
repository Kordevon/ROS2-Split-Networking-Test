#!/bin/bash

# --- Configuration (ADJUST THESE) ---
ROS_DOMAIN_ID="0"
CONTAINER_NAME="ros_container_2"
DOCKER_IMAGE="osrf/ros:humble-desktop"
# Point to the ros_eth_usb-specific profile
FAST_DDS_PROFILE_PATH="/home/kevin/ws/src/fastdds_profiles/fastdds_ros_eth_usb.xml"
WORKSPACE_HOST_PATH="/home/kevin/ws/"
ROS_DISTRO="humble"

echo "--- Starting Docker Container 2 (Node C/D) in Host Network Mode ---"
echo "Container Name: ${CONTAINER_NAME}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "Using FastDDS Profile: ${FAST_DDS_PROFILE_PATH}" # New for clarity

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
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_config.xml \
  ${DOCKER_IMAGE} \
  bash -c " \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        source /ros_ws/install/setup.bash && \
        echo 'ROS2 environment loaded. You are in ${CONTAINER_NAME}.' && \
        echo 'Container IPs (same as host): $(hostname -I)' && \
        echo 'To run Node C: ros2 run NODE_C_PKG NODE_C_EXEC' && \
        echo 'To run Node D: ros2 run NODE_C_PKG NODE_D_EXEC' && \
        exec bash \
      "

echo "Container ${CONTAINER_NAME} exited."
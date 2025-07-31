#!/bin/bash

# --- Configuration ---
ROS_DISTRO="humble"
WORKSPACE_HOST_PATH="/home/kevin/ws/"
FAST_DDS_PROFILE_PATH="/home/kevin/ws/src/fastdds_profiles/fastdds_eth0.xml"

# Container-specific configurations
CONTAINER_TYPE="${1:-controls}"  # controls or camera

case "$CONTAINER_TYPE" in
    "controls")
        CONTAINER_NAME="ros_controls_container"
        DOCKER_IMAGE="osrf/ros:humble-desktop"
        PROFILE_NAME="controls_profile"
        ROS_DOMAIN_ID="0"
        ;;
    "camera")
        CONTAINER_NAME="ros_camera_container" 
        DOCKER_IMAGE="osrf/ros:humble-desktop"
        PROFILE_NAME="camera_profile"
        ROS_DOMAIN_ID="1"
        ;;
    *)
        echo "Usage: $0 [controls|camera]"
        echo "  controls - Launch controls container (high bandwidth radio)"
        echo "  camera   - Launch camera container (low bandwidth radio)"
        exit 1
        ;;
esac

echo "--- Starting Docker Container: ${CONTAINER_NAME} ---"
echo "Container Type: ${CONTAINER_TYPE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "FastDDS Profile: ${FAST_DDS_PROFILE_PATH}"
echo "Profile Name: ${PROFILE_NAME}"

# Validate profile file exists
if [[ ! -f "${FAST_DDS_PROFILE_PATH}" ]]; then
    echo "ERROR: FastDDS profile file not found: ${FAST_DDS_PROFILE_PATH}"
    exit 1
fi

# Stop existing container
docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

# Launch container with proper FastDDS configuration
sudo docker run -it --rm \
    --network=host \
    --name ${CONTAINER_NAME} \
    --privileged \
    -v /dev:/dev \
    -v "${FAST_DDS_PROFILE_PATH}:/fastdds_eth0.xml:ro" \
    -v "${WORKSPACE_HOST_PATH}:/ros_ws:rw" \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e RMW_FASTRTPS_USE_QOS_FROM_XML=1 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_eth0.xml \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ${DOCKER_IMAGE} \
    bash -c " \
        export RMW_FASTRTPS_USE_QOS_FROM_XML=1 && \
        export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_eth0.xml && \
        export RMW_FASTRTPS_PARTICIPANT_PROFILE_NAME=${PROFILE_NAME} && \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        cd /ros_ws && \
        if [[ -f install/setup.bash ]]; then \
            source install/setup.bash; \
        else \
            echo 'No install/setup.bash found. Run colcon build first.'; \
        fi && \
        echo '=== ROS2 Environment Ready ===' && \
        echo 'Container: ${CONTAINER_NAME}' && \
        echo 'Profile: ${PROFILE_NAME}' && \
        echo 'Domain ID: ${ROS_DOMAIN_ID}' && \
        echo 'FastDDS Profile: /fastdds_eth0.xml' && \
        echo '' && \
        echo 'Network interfaces:' && \
        ip addr show | grep -E 'inet.*scope global' && \
        echo '' && \
        echo 'Test commands:' && \
        echo '  ros2 daemon start' && \
        echo '  ros2 topic list' && \
        echo '  ros2 node list' && \
        echo '' && \
        exec bash \
    "

echo "Container ${CONTAINER_NAME} exited."
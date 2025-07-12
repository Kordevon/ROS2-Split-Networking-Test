#!/bin/bash

PROFILE=${1:-camera_profile.xml}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DDS_PROFILE_PATH="$SCRIPT_DIR/$PROFILE"

docker run --rm --network=host \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/ws/src/DDS_Profiles/camera_profile.xml \
  -v "$DDS_PROFILE_PATH:/home/ws/src/DDS_profiles/$PROFILE:ro \
  ros2-multi-test \
  bash -c "source /opt/ros/humble/setup.bash && \
           source /home/ws/install/setup.bash && \
           ros2 run test_camera camera_node"

# Use a base image with ROS Humble installed.
# 'ros-base' is lighter than 'desktop' and suitable for robot deployments.
FROM osrf/ros:humble-desktop

# Set environment variables for ROS distribution and workspace path.
ENV ROS_DISTRO=humble
ENV ROS_WS=/ros_ws



# Install common build tools and Python package installer.
# Add any other system-level dependencies your packages might need here.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    git \
    wget \
    curl \
    # Add common ROS2 dev tools if needed, e.g.:
    # ros-humble-ros2-control \
    # ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# --- Workspace Setup ---
# Create the ROS workspace directory.
WORKDIR ${ROS_WS}
RUN mkdir -p src

# Copy your ROS 2 packages from the host's workspace 'src' directory into the container's '/ros_ws/src'.
# This assumes your Dockerfile is at the root of your ROS2 workspace on the host.
# This will include the source code for 'test_camer', 'test_controls', and any other packages in your src directory.
COPY ./src ${ROS_WS}/src

# --- ROSDEP Installation ---
# Initialize rosdep. This is needed to install ROS package dependencies.
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep init"

# Update rosdep database.
RUN /bin/bash -c "rosdep update"

# Install ROS dependencies for your workspace.
# This will resolve dependencies for 'test_camer', 'test_controls', and any other packages in src.
# --from-paths ${ROS_WS}/src: Specifies to look for packages in the src directory.
# --ignore-src: Ignores packages in src if they are also available as debians.
# --rosdistro ${ROS_DISTRO}: Specifies the ROS distribution.
# -y: Answer yes to all prompts.
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths ${ROS_WS}/src --ignore-src --rosdistro ${ROS_DISTRO} -y"

# --- Build Workspace ---
# Build the ROS workspace using colcon.
# This step will compile 'test_camer', 'test_controls', and all other packages in your src directory.
# --symlink-install: Creates symlinks for executables and scripts, useful for faster rebuilds during development.
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"

# --- Entrypoint and Default Command ---
# Create an entrypoint script that sources the ROS environment and your workspace.
# This ensures ROS commands work automatically when you enter the container.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "source ${ROS_WS}/install/setup.bash" >> /etc/bash.bashrc

# Set the default command to run when the container starts without a specific command.
# This drops you into a bash shell as root.
CMD ["/bin/bash"]

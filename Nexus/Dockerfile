# Use ROS 2 Jazzy Desktop as the base image
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install necessary system dependencies and ROS 2 packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for the custom nodes (e.g., Flask for dashboard)
RUN pip3 install --no-cache-dir flask --break-system-packages

# Create workspace directory
WORKDIR /ros2_ws

# Copy the source code into the workspace
# Note: In a real scenario, you would copy the src directory. 
# We copy the entire project here assuming the context is the root of the project.
COPY src/ /ros2_ws/src/

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Add sourcing to bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# Set the default command to launch bash
CMD ["bash"]

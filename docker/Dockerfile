# Red Rover Teleop - Xbox 360 Controller Container
# Provides Xbox 360 controller teleoperation for the rover
FROM ros:foxy

# Install teleop and dependencies
RUN apt-get update && apt-get install -y \
    ros-foxy-fastrtps \
    ros-foxy-rmw-fastrtps-cpp \
    ros-foxy-joy \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-ros2cli \
    ros-foxy-ros2topic \
    ros-foxy-ros2launch \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create directories
RUN mkdir -p /config /ros2_ws/src

# Copy teleop node
COPY src/xbox_teleop_node.py /ros2_ws/src/xbox_teleop_node.py
RUN chmod +x /ros2_ws/src/xbox_teleop_node.py

# Copy launch file
COPY launch/teleop.launch.py /ros2_ws/src/teleop.launch.py

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=0

# Source ROS 2 on bash startup
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "export FASTDDS_DEFAULT_PROFILES_FILE=/config/fastdds_discovery_client.xml" >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["bash"]

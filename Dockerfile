FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-visualization-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    curl \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install zenoh-bridge-ros2dds (ARM64 Linux standalone — statically links CycloneDDS)
RUN curl -sL "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.9.0/zenoh-plugin-ros2dds-1.9.0-aarch64-unknown-linux-gnu-standalone.zip" \
    -o /tmp/zenoh.zip && \
    unzip /tmp/zenoh.zip zenoh-bridge-ros2dds -d /usr/local/bin/ && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm /tmp/zenoh.zip

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

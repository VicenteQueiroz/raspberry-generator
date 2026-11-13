# Everything needed to generate raspberry fields and look at them in Gazebo:
# ROS 2 Jazzy + Gazebo Harmonic (ros_gz) + Blender, with this package built.
#
#   docker build -t raspberry-generator .
#   ./docker/run.sh demo
#
FROM osrf/ros:jazzy-desktop

ARG BLENDER_VERSION=4.2.3
ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic + ROS bridge, and the runtime libraries the Blender binary needs
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        curl xz-utils \
        libxi6 libxxf86vm1 libxfixes3 libxrender1 libxkbcommon0 libsm6 libice6 \
        libgl1 libegl1 libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Blender (headless use only: generating meshes and preview renders)
RUN curl -fsSL "https://download.blender.org/release/Blender${BLENDER_VERSION%.*}/blender-${BLENDER_VERSION}-linux-x64.tar.xz" \
        | tar -xJ -C /opt \
    && ln -s "/opt/blender-${BLENDER_VERSION}-linux-x64/blender" /usr/local/bin/blender \
    && blender --version | head -1

# this package
WORKDIR /ws
COPY . /ws/src/raspberry_field
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select raspberry_field \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# generated worlds are written here; mount a host directory to keep them
VOLUME /data
ENV QT_X11_NO_MITSHM=1
ENTRYPOINT ["/entrypoint.sh"]
CMD ["help"]

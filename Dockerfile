FROM ros:humble-ros-base-jammy
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        build-essential cmake python3-colcon-common-extensions \
        libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-msgs ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Install the Livox SDK ONLY (the driver will be mounted in later)
COPY --chown=root:root Livox-SDK2 /Livox-SDK2
RUN mkdir /Livox-SDK2/build && cd /Livox-SDK2/build \
 && cmake .. && make -j$(nproc) && make install \
 && echo '/usr/local/lib' > /etc/ld.so.conf.d/livox_lidar.conf && ldconfig

# Same entrypoint as before
WORKDIR /ws_livox
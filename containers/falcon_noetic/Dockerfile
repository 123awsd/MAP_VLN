FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      /etc/apt/sources.list \
    && apt-get -o Acquire::Retries=5 -o Acquire::https::Timeout=30 update \
    && apt-get -o Acquire::Retries=5 -o Acquire::https::Timeout=30 install -y --no-install-recommends \
    build-essential \
    git \
    python3-pip \
    libgoogle-glog-dev \
    libdw-dev \
    libdwarf-dev \
    libarmadillo-dev \
    libc++-dev \
    libc++abi-dev \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    ros-noetic-cmake-modules \
    ros-noetic-cv-bridge \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport \
    ros-noetic-laser-geometry \
    ros-noetic-nodelet \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
    cmake==3.26.4

COPY third_party/nlopt /tmp/nlopt
RUN cmake -S /tmp/nlopt -B /tmp/nlopt/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DNLOPT_PYTHON=OFF \
    -DNLOPT_OCTAVE=OFF \
    -DNLOPT_MATLAB=OFF \
    -DNLOPT_GUILE=OFF \
    -DNLOPT_SWIG=OFF \
    && cmake --build /tmp/nlopt/build --parallel 4 \
    && cmake --install /tmp/nlopt/build \
    && ldconfig \
    && rm -rf /tmp/nlopt

WORKDIR /workspace/falcon_ws/src
COPY third_party/FALCON /workspace/falcon_ws/src/FALCON
COPY ros_ws/src/pre_map_bridge /workspace/falcon_ws/src/pre_map_bridge
COPY patches/falcon/habitat_ground_viewpoint.patch /tmp/habitat_ground_viewpoint.patch
RUN patch -d FALCON -p1 < /tmp/habitat_ground_viewpoint.patch

# Habitat replaces FALCON's built-in CUDA/Open3D sensor renderers.
RUN touch FALCON/uav_simulator/camera_sensing/pointcloud_render/CATKIN_IGNORE \
    FALCON/uav_simulator/camera_sensing/mesh_render/CATKIN_IGNORE \
    FALCON/uav_simulator/map_render/CATKIN_IGNORE

WORKDIR /workspace/falcon_ws
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j4

COPY containers/falcon_noetic/nvidia-smi-compat /usr/local/bin/nvidia-smi
COPY containers/falcon_noetic/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /usr/local/bin/nvidia-smi
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM nvidia/cuda:12.4.0-base-ubuntu22.04

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    CMAKE_BUILD_PARALLEL_LEVEL=2 \
    MAKEFLAGS=-j2

RUN sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      /etc/apt/sources.list \
    && apt-get -o Acquire::Retries=5 update \
    && apt-get install -y --no-install-recommends ca-certificates curl gnupg2 \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get -o Acquire::Retries=5 update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions python3-numpy \
      libcgal-dev libgmp-dev libmpfr-dev libopencv-dev \
      ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/occusg_ws/src
COPY third_party/OccuSG/src/incremental_dude_ros2 /workspace/occusg_ws/src/incremental_dude_ros2

WORKDIR /workspace/occusg_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --parallel-workers 1 \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

COPY containers/occusg_humble/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

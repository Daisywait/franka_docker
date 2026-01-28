# 基础镜像
FROM osrf/ros:jazzy-desktop

# 环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 1. 安装基础依赖 (新增了 libpinocchio-dev)
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    ros-jazzy-pinocchio \
    libpoco-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. 初始化 rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# 3. 编译 libfranka 0.15.0
WORKDIR /tmp
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash \
    && git clone --recursive --branch 0.15.0 https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && cd /tmp && rm -rf libfranka"

# 4. 安装 MoveIt 和控制器
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# 5. 开发模式（宿主机挂载 ros2_ws/src 后在容器内构建）
WORKDIR /root/ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CMD ["/bin/bash"]

# 宿主机与 Docker 分支使用说明

本仓库构建的是 **ROS 2 Jazzy (Ubuntu 24.04)** 的 Docker 环境，但宿主机是 **Ubuntu 22.04**。
请根据运行节点的位置选择正确的 `franka_ros2` 分支。

## 在宿主机运行（Ubuntu 22.04 / ROS 2 Humble）
1. 切换到 Humble：
   ```bash
   cd /home/asus/ros2_ws/src/franka_ros2
   git checkout humble
   ```
2. 在宿主机工作空间构建：
   ```bash
   cd /home/asus/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```
3. 按需启动你的 launch。

## 在 Docker 内构建/运行（Ubuntu 24.04 / ROS 2 Jazzy）
1. 切换到 Jazzy：
   ```bash
   cd /home/asus/ros2_ws/src/franka_ros2
   git checkout jazzy
   ```
2. 把 `ros2_ws/src` 复制到 `/home/asus/franka_docker/ros2_ws/src/`，再构建镜像。

## Docker 运行（整包挂载）
```bash
docker run --rm -it \
  --net=host --ipc=host \
  --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  --device=/dev/gripper:/dev/gripper \
  -v /home/asus/cyclonedds.xml:/etc/cyclonedds/cyclonedds.xml:ro \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml \
  -v /home/asus/ros2_ws/src:/root/ros2_ws/src \
  franka-jazzy:latest
```

容器内构建（首次进入或源码有变更时执行）：
```bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

最小启动检查清单（容器内）：
1) `franka_ros2` 在 `jazzy` 分支；`fr3_robotiq_moveit_config` 在 `jazzy` 分支  
2) `ros2_robotiq_gripper` / `franka_description` / `serial` 不需要切换分支（保持默认即可）  
3) `source /root/ros2_ws/install/setup.bash` 已生效（新 shell 打开会自动加载）

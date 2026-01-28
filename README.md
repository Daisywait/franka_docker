# franka_docker 使用说明

本镜像用于 **ROS 2 Jazzy (Ubuntu 24.04)** 的 FR3 + MoveIt Servo 开发环境。
宿主机为 **Ubuntu 22.04 / ROS 2 Humble**，请注意分支与运行环境匹配。

## 1. 分支准备（宿主机）
### Humble（宿主机直接运行）
```bash
cd /home/asus/ros2_ws/src/franka/franka_ros2
git checkout humble

cd /home/asus/ros2_ws/src/moveit/fr3_robotiq_moveit_config
git checkout humble
```

### Jazzy（容器内运行）
```bash
cd /home/asus/ros2_ws/src/franka/franka_ros2
git checkout jazzy

cd /home/asus/ros2_ws/src/moveit/fr3_robotiq_moveit_config
git checkout jazzy
```

`ros2_robotiq_gripper` / `franka_description` / `serial` 保持默认分支即可。

## 2. 构建镜像（宿主机）
```bash
docker build -t franka-jazzy:latest /home/asus/franka_docker
```

## 3. 创建容器并后台运行
```bash
docker run -d --name franka-jazzy \
  --net=host --ipc=host \
  --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  --device=/dev/gripper:/dev/gripper \
  -v /home/asus/cyclonedds.xml:/etc/cyclonedds/cyclonedds.xml:ro \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml \
  -v /home/asus/ros2_ws/src:/root/ros2_ws/src \
  franka-jazzy:latest \
  tail -f /dev/null
```

## 4. 进入容器
```bash
docker exec -it franka-jazzy /bin/bash
```

## 5. 容器内构建（首次进入或源码变更时）
```bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 6. 容器内目录说明
- `/root/ros2_ws`：工作空间（源码在 `/root/ros2_ws/src`，build/install/log 在容器内生成）
- `/usr/local`：libfranka 安装结果（如 `/usr/local/lib/libfranka.so*`、`/usr/local/bin/*`）
- libfranka 源码不保留在容器内（构建阶段已清理）

## 7. Docker 数据目录
- 容器/镜像等数据位于 `/home/docker_data`（`/etc/docker/daemon.json` 的 `data-root` 指定）

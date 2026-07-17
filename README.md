# Dobot Nova2 ROS2 Driver

基于 TCP/IP 协议的 [越疆 Dobot Nova2](https://www.dobot-robots.com) 六轴协作机器人 ROS2 Humble 驱动，在 Docker 容器内运行，通过 Dashboard/Move 端口与控制器通信。

## 环境

必须使用 Docker 镜像运行，宿主机仅存放源码：

> [Mafumaful/Ubuntu2204](https://github.com/Mafumaful/Ubuntu2204) — 预装了 ROS2 Humble 的 Ubuntu 22.04 镜像。

源码通过 volume 挂载进容器的 `/home/ros/ws`，在容器内构建和运行。

```bash
cd /path/to/Ubuntu2204
docker compose run --rm ros2 bash
```

## 构建

```bash
colcon build --symlink-install --packages-up-to dobot_ros2
source install/setup.bash
```

或：

```bash
make build
```

## 启动

```bash
make driver     # 纯 driver
make bringup    # driver + robot_state_publisher（TF）
make rviz       # driver + robot_state_publisher + RViz
```

## 常用命令

| 命令 | 作用 |
|---|---|
| `make state` | 读取机器人状态 |
| `make joints` | 读取关节角 |
| `make tcp` | 读取 TCP 位姿 |
| `make errors` | 查看报警 |
| `make clear` | 清除报警 |
| `make enable` / `make disable` | 上/下使能 |
| `make movej J:='[...]'` | 关节运动 |
| `make movel P:='[...]'` | 直线运动 |
| `make movep P:='[...]'` | 点位姿运动 |
| `make tf` | 查看 TF topic |
| `make frames` | 生成 TF 帧图 |
| `make services` | 列出所有 service |

运动参数默认值：`SPEED=2 ACC=2 WAIT=true TIMEOUT=20`。

## 包结构

| 包 | 内容 |
|---|---|
| `dobot_interfaces` | 自定义 srv（MoveCommand / GetRobotState 等） |
| `dobot_description` | URDF 模型和 STL mesh |
| `dobot_ros2` | 驱动节点、launch 文件、RViz 配置 |

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
make bringup    # driver + robot_state_publisher（TF）+ 可选手眼 static TF
make rviz       # bringup + RViz
make control-ui # driver + robot_state_publisher + Web 控制台 + 可选手眼 static TF
```

手眼 static TF 默认读取 `HANDEYE_STATIC_TF_FILE=$(WS)/handeye_result.yaml`。如果文件不存在，bringup 会跳过相机 TF，不影响机械臂 driver 启动。

## 常用命令

| 命令 | 作用 |
|---|---|
| `make state` | 读取机器人状态 |
| `make joints` | 读取关节角 |
| `make tcp` | 读取 TCP 位姿 |
| `make errors` | 查看报警 |
| `make clear` | 清除报警 |
| `make enable` / `make disable` | 上/下使能 |
| `make estop` | 软件急停，调用 `EmergencyStop()` |
| `make control-ui` | 启动完整 Web 控制台，可查看/复制 Joint 和 TCP、下发移动、夹爪和示教命令 |
| `make control-ui-only` | 只启动 Web 控制台，连接已有 driver |
| `make gripper-init` | 初始化 AG 夹爪 |
| `make gripper-state` | 读取夹爪初始化、夹持和位置状态 |
| `make gripper-open` / `make gripper-close` | 张开/闭合夹爪 |
| `make gripper-move GRIPPER_OPENING_MM:=50 GRIPPER_FORCE_N:=80` | 按开口宽度和夹持力控制夹爪 |
| `make camera` | 通过 `dobot_camera` 封装启动 Orbbec Gemini305，默认调用官方 `gemini_330_series.launch.py` |
| `make camera-topics` | 查看 `/camera` 下的相机 topic |
| `make camera-info` | 读取一次 `/camera/color/camera_info` |
| `make handeye-check` | 检查手眼标定需要的 Dobot 和相机 topic |
| `make handeye-capture` | 创建手眼标定数据集，稳定后按 Enter 保存样本、图像和位姿 |
| `make handeye-solve DATASET:=...` | 根据数据集求解 `Link6 -> camera_color_optical_frame` |
| `make handeye-validate DATASET:=...` | 验证各样本反推的固定标定板位姿误差 |
| `make handeye-diagnose DATASET:=...` | 对比多种手眼算法，并逐个剔除样本检查可疑点 |
| `make handeye-tf` | 发布手眼标定结果 static TF |
| `make handeye-board-tf` | 实时识别 ChArUco 标定板，发布 `camera_color_optical_frame -> handeye_board` |
| `make teach-start TRAJ:=demo` | 进入拖拽示教并开始录点 |
| `make teach-stop` | 停止示教并保存轨迹 |
| `make teach-replay TRAJ:=demo` | 使用 `movej` 回放轨迹 |
| `make teach-replay-servoj TRAJ:=demo` | 使用 `ServoJ` 平滑回放轨迹 |
| `make teach-list` | 列出已保存轨迹 |
| `make teach-delete TRAJ:=demo` | 删除轨迹 |
| `make teach-status` | 查看示教录制状态 |
| `make movej J:='[...]'` | 关节运动 |
| `make movel P:='[...]'` | 直线运动 |
| `make movep P:='[...]'` | 点位姿运动 |
| `make tf` | 查看 TF topic |
| `make topics` | 查看常用状态 topic |
| `make frames` | 生成 TF 帧图 |
| `make services` | 列出所有 service |

运动参数默认值：`SPEED=2 ACC=2 WAIT=true TIMEOUT=20`。

Web 控制台默认地址：`http://localhost:8080`。可用 `CONSOLE_PORT` 覆盖端口。

示教命令变量：`TRAJ` 指定轨迹名，`OVERWRITE=true` 允许覆盖同名轨迹，`REPLAY_MODE` 可覆盖回放模式。

## 常用 Topic

| Topic | 类型 | 作用 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节角，供 TF/RViz 和下游节点订阅 |
| `/tcp_pose` | `std_msgs/msg/Float64MultiArray` | TCP 位姿 `[x,y,z,rx,ry,rz]` |
| `/dobot_state` | `dobot_interfaces/msg/DobotState` | 机器人模式、使能、运行、报警等状态 |
| `/gripper_state` | `dobot_interfaces/msg/GripperStatus` | 夹爪初始化、夹持、开口和是否夹住物体 |

## 手眼标定

当前流程按腕部相机实现，即 eye-in-hand，目标是求解固定变换：

```text
Link6 -> camera_color_optical_frame
```

Gemini305 底层 driver 使用 Orbbec 官方 ROS2 包 `orbbec_camera`，不在本仓库内二次改造。工控机上官方 workspace 当前放在：

```text
~/orbbec_305
```

Makefile 默认会在存在时自动 source：

```text
ORBBEC_WS=$(HOME)/orbbec_305
```

也就是：

```bash
source ~/orbbec_305/install/setup.bash
```

因此工控机上按默认目录放置并构建官方驱动后，直接 `make build`、`make camera` 即可。若官方 workspace 放在其它位置，可用 `ORBBEC_WS:=...` 覆盖。

本仓库只提供 `dobot_camera` 这个薄封装包，用来固定现场启动入口和常用参数。手眼工具消费相机已经发布的 color 图像和内参：

```text
/camera/color/image_raw
/camera/color/camera_info
```

手眼标定工具在独立包 `dobot_handeye` 中，Makefile 命令保持不变。

现场操作：

```bash
make build
make bringup
```

另开终端启动相机：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
make camera
```

确认 topic：

```bash
make camera-topics
make handeye-check
```

固定 ChArUco 标定板，不要让板子移动。移动机械臂末端，让腕部相机从不同位置和角度看到整块标定板。每到一个稳定姿态，执行采样终端里按一次 Enter：

```bash
make handeye-capture
```

启动后终端会打印本次数据集目录，例如：

```text
handeye dataset: handeye_datasets/20260723_153012
```

每按一次 Enter 会保存一组样本，包括原始 color 图、debug 图、`base_link -> Link6`、`board -> camera_color_optical_frame`、相机内参和检测角点数量：

```text
handeye_datasets/20260723_153012/
  dataset.yaml
  samples/
    sample_001.json
    sample_001_color.png
    sample_001_debug.png
```

建议采 15-30 组。每组需要能清楚看到标定板的大部分区域，不是只看某一个格子。姿态要有明显旋转变化：正对、左偏、右偏、上偏、下偏、近一点、远一点、绕相机光轴旋转一点。

采样完成后输入 `q` 退出采样。把终端打印的数据集目录填到 `DATASET`：

```bash
make handeye-solve DATASET:=handeye_datasets/20260723_153012
make handeye-validate DATASET:=handeye_datasets/20260723_153012
make handeye-diagnose DATASET:=handeye_datasets/20260723_153012
make handeye-tf DATASET:=handeye_datasets/20260723_153012
make handeye-board-tf
```

若希望 `make bringup`、`make rviz`、`make control-ui` 默认发布当前相机 TF，可把最终结果复制到工作区固定文件：

```bash
cp handeye_datasets/20260723_153012/result.yaml handeye_result.yaml
```

求解和验证会写入：

```text
handeye_datasets/20260723_153012/result.yaml
handeye_datasets/20260723_153012/validation.yaml
handeye_datasets/20260723_153012/diagnose.yaml
```

`handeye-validate` 会输出每组样本反推出的 `base_link -> board` 一致性误差。标定板固定不动时，误差越小说明手眼结果越稳定；重点看 `translation_rms_mm`、`translation_max_mm`、`rotation_rms_deg` 和 `worst_sample_id`。

`handeye-diagnose` 用同一份数据集分别尝试 `TSAI`、`PARK`、`HORAUD`、`ANDREFF`、`DANIILIDIS`，并做 leave-one-out 检查：每次只移除一个样本重新求解和验证。优先看 `best_method`、`methods` 和 `leave_one_out` 前几项；如果移除某个样本后 RMS 明显降低，这个样本才更像真正坏点。

`handeye-board-tf` 用在线相机画面实时识别标定板，并发布动态 TF `camera_color_optical_frame -> handeye_board`。配合 `make rviz` 和 `make handeye-tf` 可以在 RViz 里观察标定板 frame：机械臂从不同角度看同一块固定板时，`handeye_board` 在 `base_link` 下应基本不动。

如果检测不到标定板，优先检查光照、反光、画面模糊、距离过远、标定板没有完整入画。CC200-15-11.25 当前按 `DICT_5X5_100`、`12 x 9`、`15mm / 11.25mm` 配置。

## 配置参数

默认配置文件：

```text
src/dobot_ros2/config/dobot_ros2.yaml
```

当前配置按 Nova 2 写入厂家手册 V1.5 的标称参数：

| 参数 | Nova 2 默认值 |
|---|---|
| `robot_model` | `Nova 2` |
| `rated_payload_kg` | `2.0` |
| `workspace_radius_mm` | `625.0` |
| `max_tcp_speed_mps` | `1.6` |
| `repeatability_mm` | `0.05` |
| `max_joint_speed_deg_s` | 六轴均 `135.0` |
| `joint_zero_deg` | `[0, 0, 0, 0, 0, 0]` |
| `joint_lower_limits_deg` | `[-360, -180, -156, -360, -360, -360]` |
| `joint_upper_limits_deg` | `[360, 180, 156, 360, 360, 360]` |
| `teach_trajectory_dir` | `/home/ros/ws/trajectories` |
| `teach_sample_rate_hz` | `5.0` |
| `teach_min_joint_delta_deg` | `0.5` |
| `teach_min_tcp_delta_mm` | `1.0` |
| `teach_replay_speed` / `teach_replay_acc` | `10` / `10` |
| `teach_replay_mode` | `movej` |
| `teach_servoj_rate_hz` | `33.0` |
| `teach_servoj_t` / `teach_servoj_lookahead_time` / `teach_servoj_gain` | `0.1` / `50.0` / `500.0` |
| `gripper_enabled` / `gripper_transport` | `true` / `dobot_modbus` |
| `gripper_modbus_ip` / `gripper_modbus_port` | `127.0.0.1` / `60000` |
| `gripper_port` | `/dev/ttyUSB0`（仅 `local_serial` 使用） |
| `gripper_baudrate` / `gripper_slave_id` | `115200` / `1` |
| `gripper_stroke_mm` / `gripper_max_force_n` | `95.0` / `160.0` |
| `gripper_default_force_percent` | `50` |
| `gripper_state_rate_hz` | `2.0` |
| `handeye.image_topic` | `/camera/color/image_raw` |
| `handeye.camera_info_topic` | `/camera/color/camera_info` |
| `handeye.base_frame` / `handeye.flange_frame` | `base_link` / `Link6` |
| `handeye.camera_frame` | `camera_color_optical_frame` |
| `handeye.board.dictionary` | `DICT_5X5_100` |
| `handeye.board.squares_x` / `handeye.board.squares_y` | `12` / `9` |
| `handeye.board.square_length_m` / `handeye.board.marker_length_m` | `0.015` / `0.01125` |

`joint_limit_check` 默认开启。`movej` 会在下发前检查目标关节角，`movel`/`movejp`/`movep` 会检查 IK 解出的关节角。若现场控制器配置了更小的软件限位、使用 Nova 5、或工具负载发生变化，需要同步修改这个 YAML。

AG-160-95-W-S 夹爪通过机械臂末端 RS485 接入，默认使用 Dobot 控制器的 Modbus-RTU 转发控制。位置命令支持 `GRIPPER_OPENING_MM`，也支持 `GRIPPER_POS` 千分比；夹持力支持 `GRIPPER_FORCE` 百分比或 `GRIPPER_FORCE_N` 牛顿值，最终会映射到厂家 20-100% 力值寄存器。夹持状态 `2` 表示夹住物体，`3` 表示物体掉落。若改为 USB-RS485 直连电脑调试，可将 `gripper_transport` 改为 `local_serial` 并配置 `gripper_port`。

## 包结构

| 包 | 内容 |
|---|---|
| `dobot_interfaces` | 自定义 msg/srv（DobotState / GripperStatus / MoveCommand 等） |
| `dobot_description` | URDF 模型和 STL mesh |
| `dobot_ros2` | 驱动节点、launch 文件、RViz 配置 |
| `dobot_handeye` | 可选手眼标定工具、在线 board TF 验证 |
| `dobot_camera` | Orbbec Gemini305 官方 driver 的 launch 封装 |

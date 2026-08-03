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

手眼 static TF 默认读取 `HANDEYE_STATIC_TF_FILE=$(WS)/handeye_result.yaml`。如果文件不存在，bringup 会跳过相机 TF，不影响机械臂 driver 启动。默认发布到 Orbbec 相机树根 `camera_link`，再由官方 driver 发布 `camera_link -> camera_color_optical_frame` 等相机内部 TF。

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
| `make recover-limit` | 交互式恢复关节限位抱死，释放抱闸前后都需要人工确认 |
| `make drag-start` / `make drag-stop` | 只开启/关闭拖拽模式，不录制轨迹 |
| `make control-ui` | 启动完整 Web 控制台，可查看/复制 Joint 和 TCP、下发移动、夹爪和示教命令 |
| `make control-ui-only` | 只启动 Web 控制台，连接已有 driver |
| `make system` | 一键启动 bringup、双相机、手柄和数据采集，并生成独立运行日志 |
| `make logs-latest` | 输出最近一次 `make system` 的日志目录 |
| `make gripper-init` | 初始化 AG 夹爪 |
| `make gripper-state` | 读取夹爪初始化、夹持和位置状态 |
| `make gripper-open` / `make gripper-close` | 张开/闭合夹爪 |
| `make gripper-move GRIPPER_OPENING_MM:=50 GRIPPER_FORCE_N:=80` | 按开口宽度和夹持力控制夹爪 |
| `make camera` | 同时启动腕部 Orbbec Gemini305 和全局 RealSense |
| `make camera-wrist` / `make camera-global` | 只启动指定相机，便于单独调试 |
| `make camera-view` | 只打开腕部和全局相机画面，不启动或停止相机驱动 |
| `make camera-topics` / `make camera-info` | 查看两台相机的 Topic 或读取两份 CameraInfo |
| `make camera-check` | 分别检查腕部和全局彩色图像帧率 |
| `make handeye-check` | 检查手眼标定需要的 Dobot 和相机 topic |
| `make handeye-capture` | 创建手眼标定数据集，稳定后按 Enter 保存样本、图像和位姿 |
| `make handeye-solve DATASET:=...` | 根据数据集求解 `Link6 -> camera_color_optical_frame` |
| `make handeye-validate DATASET:=...` | 验证各样本反推的固定标定板位姿误差 |
| `make handeye-diagnose DATASET:=...` | 对比多种手眼算法，并逐个剔除样本检查可疑点 |
| `make handeye-tf` | 发布手眼标定结果 static TF |
| `make handeye-board-tf` | 实时识别 ChArUco 标定板，发布 `camera_color_optical_frame -> handeye_board` |
| `make keyboard` | 启动键盘笛卡尔小步控制，按键发布到 `/keyboard/input` 后由 teleop 节点调用移动和夹爪 service |
| `make keyboard-jog KEYBOARD_DEV:=/dev/input/eventX` | 启动工控机本机键盘连续点动，按下运动、松开 `MoveJog()` 停止 |
| `make keyboard-input` / `make keyboard-teleop` | 分开启动键盘输入节点和执行节点，便于调试 |
| `make joy` | 启动 ROS2 `joy_node`、Dobot joy teleop 和手柄遥操作数据采集节点 |
| `make joy-teleop` | 只启动 Dobot joy teleop，连接已有 `/joy` topic |
| `make lerobot-setup` | 创建隔离 Python 3.12 环境并安装固定版本的官方 LeRobot v3 工具 |
| `make data-start` / `make data-stop` | 调试用：通过 service 开始/停止或确认手柄遥操作数据采集 |
| `make data-accept` / `make data-reject` | 接受或拒绝待审核 episode；拒绝只标记原始数据，不删除文件 |
| `make data-set-start` | 保存当前反馈为 ServoP 采集标准起点，不运动 |
| `make data-prepare` | 显式回到保存的起点并校验夹爪、关节反馈 |
| `make servo-data-lerobot-validate` | 只读验证独立 ServoP LeRobot v3 数据集 |
| `make data-status` | 查看当前任务、采集阶段、同步差、样本数、队列和错误数 |
| `make data-task TASK:="..."` | 持久保存任务描述，下一次 Start 自动读取，不需要重启 joy |
| `make data-validate EPISODE:=...` | 检查 episode 元数据、双相机图片、同步差和训练字段 |
| `make data-lerobot-export EPISODE:=...` | LeRobot 导出失败排障后，重新提交保留的完整原始 episode |
| `make data-lerobot-validate` | 使用官方 LeRobot 加载器检查整个训练数据集和第一帧 |
| `make move-jog AXIS:=X+` | 手动调用 `MoveJog(X+)`，用于调试点动方向 |
| `make jog-stop` | 手动调用 `MoveJog()` 停止点动 |
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

## π0.5 训练与实机部署

训练在 OpenPI 的 GPU Docker 容器中执行，机械臂、相机和真机 policy 都从本仓库启动。下面的
流程只需要替换 `DATASET_DIR`（训练）或 `CHECKPOINT_DIR`/`POLICY_CONFIG`（部署）；训练细节和
历史记录见 OpenPI 的 `docs/dobot_pi05_docker_training.md`，部署联调记录见
`docs/pi05_policy_deployment.md`。

### 1. 训练一个新数据集

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
OPENPI_DIR=/home/ps/DZK_repos/openpi
DATASET_DIR=/home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver/data/servo_p_v2/lerobot_pi05_servo_p_v2
EXP_NAME=dobot_pen_box_servo_p_action_only_v1_long

cd "$OPENPI_DIR"
export OPENPI_DOBOT_DATASET_DIR="$DATASET_DIR"
docker compose -f compose.yaml -f compose.gpu.yaml -f compose.dobot.yaml up -d openpi
docker compose -f compose.yaml -f compose.gpu.yaml -f compose.dobot.yaml exec openpi \
  zsh -lc 'source /usr/local/share/openpi/functions.zsh && \
    opic-norm --config-name pi05_dobot_pen_box_servo_p_action_only && \
    opic-train pi05_dobot_pen_box_servo_p_action_only \
      --exp-name='"$EXP_NAME"' --overwrite'
```

训练日志、`metrics.csv`、`training_curves.png` 和 checkpoint 都写入
`/home/ps/openpi-docker-data/`。长时训练建议在 `exec` 命令后加 `-d -T` 并将 stdout 重定向到
`openpi-docker-data/wandb/`，用 `tail -f` 观察；需要停止时在 OpenPI 仓库执行
`docker compose ... exec openpi pkill -f scripts/train.py`。

### 2. 部署一个 checkpoint

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
export OPENPI_REPO_DIR=/home/ps/DZK_repos/openpi
export OPENPI_POLICY_CONFIG=pi05_dobot_pen_box_servo_p_action_only
export OPENPI_CHECKPOINT_DIR=/home/ps/openpi-docker-data/checkpoints/pi05_dobot_pen_box_servo_p_action_only/dobot_pen_box_servo_p_action_only_v1_long/135000
export OPENPI_CHECKPOINT_CONTAINER_DIR=/workspace/checkpoints/pi05_dobot_pen_box_servo_p_action_only/dobot_pen_box_servo_p_action_only_v1_long/135000
make policy-real
```

默认起始位是 `data/servo_p_v2/servo_p_start_pose.json`。脚本会启动或复用 OpenPI Policy
Server，跳过已安装 ROS 包的重复构建，初始化夹爪并回到起点，然后进入暖会话：

| 按键 | 动作 |
|---|---|
| `r` | 停止当前 episode，打开夹爪、回起点并重新运行 |
| `q` / `Ctrl-C` | 停止 policy、停止运动并退出 |

不要在同一时间运行 `make system`、`make joy` 或其他运动节点。首次换 checkpoint 先运行
`make policy-dry-run`，确认双相机、state、Policy Server 和 `(16, 7)` action 均正常，再运行
`make policy-real`。

### 3. 数据、训练和部署日志

运行数据统一位于 `data/`；构建产物 `log/` 和运行时日志 `logs/` 保持分开：

```text
data/collections/move_jog/ 原始 MoveJog episode 和 tape LeRobot 数据集
data/collections/servo_p_v1/ 历史 ServoP 原始 episode 和 LeRobot 数据集
data/collections/servo_p_v2/ 当前 pen-box 原始 episode、起始位和 LeRobot 数据集
data/handeye/              手眼标定数据、样本和结果
data/trajectories/        示教轨迹
logs/system/run_*/        make system 的 launch、health、events 和节点日志
logs/policy/              policy JSONL、可读文本日志和观测 artifacts
logs/ros_policy_*/        policy launch 的 ROS 底层日志
log/                      colcon 构建日志（工具默认目录，不与运行日志混用）
```

`logs/policy/*.jsonl` 是机器可解析的完整记录；同名 `.log` 是按时间、事件和关键字段整理的
人类可读版本，颜色只用于终端输出，不污染 JSONL。异常时优先查看最新文件：

```bash
find logs/policy -maxdepth 1 -type f -printf '%T@ %p\n' | sort -n | tail
make logs-latest
```

Web 控制台默认地址：`http://localhost:8080`。可用 `CONSOLE_PORT` 覆盖端口。

示教命令变量：`TRAJ` 指定轨迹名，`OVERWRITE=true` 允许覆盖同名轨迹，`REPLAY_MODE` 可覆盖回放模式。

限位抱死恢复使用：

```bash
make recover-limit
```

该命令会直接连接 Dashboard `29999`，执行 `GetErrorID()`、`DisableRobot()`、`ClearError()`、`RobotMode()`，并从报警表自动识别限位关节。释放抱闸前终端会要求按 Enter 确认；你手动把关节移出限位后，再按 Enter，程序会执行 `BrakeControl(N,0)` 重新抱闸并打印最终状态。如果自动识别失败，手动指定：

```bash
make recover-limit JOINT:=6
```

恢复脚本不会自动重新使能。确认姿态、线缆和人员安全后，再执行 `make enable` 或用手柄 Y 键 enable。

键盘控制默认每次按键只发送一个小步 TCP 目标，默认步长 `KEYBOARD_STEP_MM=5.0`，旋转步长 `KEYBOARD_ROT_STEP_DEG=2.0`，默认运动 service 是 `KEYBOARD_MOTION_SERVICE=movep`。`make keyboard` 会先调用一次 `/gripper_init`，可用 `KEYBOARD_GRIPPER_INIT:=false` 跳过。可按现场情况降低步长和速度：

```bash
make keyboard KEYBOARD_STEP_MM:=2 SPEED:=1 ACC:=1
```

按键映射：

| 按键 | 作用 |
|---|---|
| `w` / `s` | TCP 沿 `base_link` 的 x 正/负方向移动 |
| `a` / `d` | TCP 沿 `base_link` 的 y 正/负方向移动 |
| `r` / `f` | TCP 沿 z 正/负方向移动 |
| `z` / `x` | TCP 绕 x 轴正/负方向旋转 |
| `t` / `g` | TCP 绕 y 轴正/负方向旋转 |
| `c` / `v` | TCP 绕 z 轴正/负方向旋转 |
| `space` | 夹爪张开/闭合切换 |
| `e` | 软件急停，调用 `/emergency_stop` |
| `q` | 仿真 reset 占位；真机不执行 reset |
| `ESC` | 退出键盘输入节点 |

键盘 teleop 会先检查 `/dobot_state`，机器人未使能、报警、反馈无效或未连接时会拒绝移动。状态允许后读取当前 `/get_tcp_pose`，再叠加小步增量并调用 `/movep`。目标会先经过 `keyboard.workspace_*` 工作空间限制检查，驱动侧 IK/关节限位检查仍然保留为第二层保护。上一条键盘命令未完成时，新按键会被忽略，避免命令队列堆积。`e` 急停不受 busy 状态限制。

如果使用工控机本机键盘做连续点动，用：

```bash
make keyboard-jog KEYBOARD_DEV:=/dev/input/eventX
```

`keyboard-jog` 不使用终端字符输入，而是读取 Linux input event，因此能区分按下和松开：

```text
按下 w -> MoveJog(X+)
松开 w -> MoveJog()
```

默认映射和 `make keyboard` 一致，`KEYBOARD_JOG_COORD_TYPE=0` 表示用户坐标系，`1` 表示工具坐标系。节点退出或析构时会主动发送一次 `MoveJog()`，避免点动命令悬挂。`Ctrl+C` 在 `make keyboard-jog` 中按一次只停止点动，终端会提示再次按 `Ctrl+C` 才退出；`Esc` 仍然用于正常停止输入。

查找工控机键盘设备：

```bash
ls -l /dev/input/by-id/
```

优先使用带 `kbd` 的软链接，例如：

```bash
make keyboard-jog KEYBOARD_DEV:=/dev/input/by-id/usb-xxx-event-kbd
```

如果没有权限读取 input 设备，需要把当前用户加入 `input` 组后重新登录，或临时用有权限的终端启动。

手柄控制提供两种模式：

- `move_jog`：默认稳定模式。一次只选择输入幅值最大的轴，由控制器内部持续点动，
  适合日常操作和排障，不支持斜线或多个 TCP 轴同时运动。
- `servo_p`：多轴连续比例模式。以 33 Hz 发送官方 `ServoP` 六维 TCP 目标，支持
  X/Y 斜线、平移与升降、平移与旋转等组合输入。

首次验证时关闭相机和数据采集，减少其它进程对控制周期的影响。两种模式分别使用：

```bash
# 稳定单轴模式
JOY_CONTROL_MODE=move_jog \
SYSTEM_START_CAMERA=false \
SYSTEM_START_DATA_COLLECTION=false \
make system

# 多轴连续模式
JOY_CONTROL_MODE=servo_p \
SYSTEM_START_CAMERA=false \
SYSTEM_START_DATA_COLLECTION=false \
make system
```

完成实机控制验证后，直接运行 `make system` 会按默认 `move_jog` 模式同时启动
bringup、双相机、手柄和数据采集；需要使用多轴控制采集数据时运行：

```bash
JOY_CONTROL_MODE=servo_p make system
```

`make joy` 只用于连接已经启动的机器人 driver；它会启动 `joy_node`、手柄 teleop
和数据采集节点，但不会替代 `make driver`、`make bringup` 或 `make system`。

### 工控机启动与检查

手柄必须在启动 `joy_node` 的同一系统环境中可见。先在工控机宿主机检查设备：

```bash
ls -l /dev/input/js* /dev/input/by-id/
```

优先使用 `/dev/input/by-id/` 下稳定的手柄软链接；没有软链接时使用 `/dev/input/js0`。如果 ROS2 在 Docker 容器内运行，还必须把对应 `/dev/input` 设备映射进容器，并保证容器用户有读取权限。正常采集先设置任务，再一键启动：

```bash
make data-task TASK:="pick up the tape roll"
make system
```

终端出现 `SYSTEM READY: joystick control and collection available` 后才开始操作。默认 `ROBOT_MODE=bringup`，同时启动完整机械臂 bringup、腕部/全局相机、手柄和采集节点，但不打开 GUI：

```bash
make system SYSTEM_VIEW=true       # 同时打开双相机画面
make system ROBOT_MODE=driver      # 只使用基础 driver，不启动完整 bringup
make system ROBOT_MODE=external    # 连接已经由其他进程启动的机械臂驱动
```

每次运行自动生成 `logs/run_YYYYMMDD_HHMMSS_PID/`。终端强制启用彩色日志级别，`launch.log` 原样保存带 ANSI 颜色的完整输出；`manifest.json` 保存版本、任务、原始/LeRobot 目录、`repo_id` 和启动配置；`events.jsonl` 保存机器人状态、夹爪初始化、手柄动作和采集审核结果；`health.jsonl` 每秒保存机器人/相机/手柄就绪状态、两路实际帧率和已发现节点。

`make system` 使用进程锁防止同一用户重复启动。若提示 `another make system is already
running`，应回到原终端继续使用或先用 `Ctrl+C` 正常结束原系统；不要并行启动两套
相机驱动，否则 RealSense 可能在内核层反复断开重连。

`nodes.jsonl` 聚合 `/rosout`，每条明确记录节点名、级别、时间、消息和源码位置；`nodes/<节点名>.log` 按节点拆分，日常排障优先查看这里。`ros/` 是 ROS2 底层日志，Python console entrypoint 会被 rclpy 按解释器进程名写成 `python3_<PID>_<时间>.log`，它不是未知节点，也不是采集数据；保留该目录只是为了底层排障。`make logs-latest` 可定位最近一次运行目录。

需要拆分排障时，仍可按以下顺序分别启动：

```bash
# 终端 1：机器人驱动
make driver

# 终端 2：腕部和全局彩色相机
make camera

# 可选：设置或更换当前任务；设置一次后会持久保存
make data-task TASK:="pick up the tape roll"

# 终端 3：手柄、遥操作和数据采集
make joy
```

需要观察画面时另开终端运行 `make camera-view`。该命令只订阅图像：只启动一台相机时显示已有画面，另一窗口等待；两台相机都启动时同时显示两路。先运行 viewer、后启动相机也可以，关闭 viewer 不影响驱动和采集。

启动后先不要按 LB，通过以下命令确认输入、机器人和相机数据持续更新：

```bash
ros2 topic hz /joy
ros2 topic echo /joy --once
make state
make camera-info
make data-status
```

确认人员、线缆和工作空间安全后，再短按 Y 使能。`make joy-teleop` 只启动遥操作节点，不启动 `joy_node` 和数据采集节点，因此单独使用它时 Start/Back 不会生成 episode，除非已经另行启动 `/data_collection/*` 服务。

默认参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `JOY_DEV` | `/dev/input/js0` | 手柄设备 |
| `JOY_TOPIC` | `/joy` | `sensor_msgs/msg/Joy` topic |
| `JOY_DEADMAN_BUTTON` | `4` | deadman 按钮，默认 LB，必须按住才允许 jog |
| `JOY_ESTOP_BUTTON` | `1` | 急停按钮，默认 B |
| `JOY_TOGGLE_ENABLE_BUTTON` | `3` | enable/disable 切换按钮，默认 Y |
| `JOY_TOGGLE_DRAG_BUTTON` | `5` | 拖拽模式开关按钮，默认 RB |
| `JOY_COLLECTION_PREPARE_HOLD_SEC` | `1.5` | Start/X 长按回位或保存起始位的最短持续时间 |
| `JOY_DEADZONE` | `0.25` | 摇杆死区 |
| `JOY_CONTROL_MODE` | `move_jog` | `move_jog` 为兼容单轴点动；`servo_p` 为六维连续比例控制 |
| `JOY_RESPONSE_EXPONENT` | `1.2` | `servo_p` 摇杆响应曲线；兼顾中心微调与低延迟响应 |
| `JOY_COORD_TYPE` | `0` | `MoveJog` 坐标系，`0` 为用户坐标系，`1` 为工具坐标系 |
| `JOY_AUTOREPEAT_RATE` | `50.0` | `joy_node` 重复发布频率，提高连续点动顺滑度 |
| `JOY_GRIPPER_INIT` | `true` | 启动手柄控制前先尝试初始化夹爪 |
| `JOY_GRIPPER_STEP_MM` | `2.0` | 保留参数；当前 LT/RT 默认按模拟点动处理 |
| `JOY_GRIPPER_STOP_LEAD_MM` | `3.0` | LT/RT 松开时的停止补偿距离，用于抵消读反馈到写目标之间的运动延迟 |
| `JOY_GRIPPER_FORCE` | `50` | 手柄夹爪命令的默认力百分比 |
| `JOY_ENABLE_RUMBLE` | `true` | 夹爪检测到物体时尝试发送手柄震动反馈 |
| `JOY_JOINT_LIMIT_MARGIN_DEG` | `5.0` | 任一关节距离软限位小于该角度时停止 jog |
| `JOY_DATASET_ROOT` | `$(WS)/data/collections/move_jog` | 手柄遥操作训练 episode 保存目录 |
| `JOY_DATA_SAMPLE_RATE_HZ` | `10.0` | 以同步图像对为基准保存训练 step 的频率 |
| `JOY_MAX_IMAGE_SKEW_SEC` | `0.05` | 腕部与全局图像允许的最大时间戳差 |
| `JOY_TASK_FILE` | `$(JOY_DATASET_ROOT)/current_task.txt` | `make data-task` 持久保存任务文本的位置 |
| `JOY_LEROBOT_ENABLED` | `true` | 审核接受后追加 LeRobot v3 episode；失败时保持 pending 以便重试 |
| `JOY_LEROBOT_PYTHON` | `$(WS)/.venv-lerobot/bin/python` | 与 ROS Python 3.10 隔离的 LeRobot Python 3.12 |
| `JOY_LEROBOT_DATASET_ROOT` | `$(JOY_DATASET_ROOT)/lerobot_tape_pi05` | 通过审核后持续追加的 LeRobot v3 训练数据集根目录 |
| `JOY_LEROBOT_REPO_ID` | `local/dobot_nova2_tape_pi05` | LeRobot 逻辑数据集身份；上传 Hub 前改为 `用户/数据集` |
| `JOY_DATA_REJECT_HOLD_SEC` | `2.0` | Back 长按达到该时长后拒绝 episode，不提交训练集 |
| `JOY_LIMIT_RECOVERY_HOLD_SEC` | `3.0` | Back+Start 进入限位恢复前的长按时间 |
| `JOY_LIMIT_RECOVERY_TIMEOUT_SEC` | `10.0` | 手柄侧抱闸释放最长时间 |
| `JOY_X_AXIS_INDEX` | `1` | 左摇杆前后轴索引 |
| `JOY_Y_AXIS_INDEX` | `0` | 左摇杆左右轴索引 |
| `JOY_RX_AXIS_INDEX` | `6` | DPad 左右轴索引，默认映射到 Rx |
| `JOY_RY_AXIS_INDEX` | `7` | DPad 上下轴索引，默认映射到 Ry |
| `JOY_X_AXIS_SIGN` | `-1.0` | 左摇杆上下到 X 方向的符号，方向反了改成 `1.0` |
| `JOY_Y_AXIS_SIGN` | `-1.0` | 左摇杆左右到 Y 方向的符号，方向反了改成 `1.0` |
| `JOY_Z_AXIS_SIGN` | `1.0` | 右摇杆上下到 Z 方向的符号，方向反了改成 `-1.0` |
| `JOY_RZ_AXIS_SIGN` | `-1.0` | 右摇杆左右到 Rz 方向的符号，方向反了改成 `1.0` |
| `JOY_RX_AXIS_SIGN` | `-1.0` | DPad 左右到 Rx 方向的符号，方向反了改成 `1.0` |
| `JOY_RY_AXIS_SIGN` | `-1.0` | DPad 上下到 Ry 方向的符号，方向反了改成 `1.0` |

默认映射：

| 手柄输入 | 功能 |
|---|---|
| 按住 LB | 允许机械臂连续点动 |
| 左摇杆上下 | `X+` / `X-`，末端前后 |
| 左摇杆左右 | `Y+` / `Y-`，末端左右 |
| 右摇杆上下 | `Z+` / `Z-`，末端升降 |
| 右摇杆左右 | `Rz+` / `Rz-`，末端绕 Z 旋转 |
| DPad 左右 | `Rx+` / `Rx-`，末端绕 X 旋转 |
| DPad 上下 | `Ry+` / `Ry-`，末端绕 Y 旋转 |
| A | 夹爪开/关切换 |
| LT / RT | 按住夹爪持续关闭 / 打开，松开时主动读取实时开口并发送保持目标 |
| B | 急停 |
| X 短按 | 清除报警 |
| X 长按 1.5 秒后松开 | 保存当前反馈为采集起始位，不运动 |
| Y | enable / disable 切换；录制期间忽略，使用 Back 结束 episode 或 B 急停 |
| RB | 开启 / 关闭拖拽模式，不录制轨迹 |
| Start 长按 1.5 秒后松开 | 回到已保存的采集起点并校验；短按仍开始采集 |
| Start 单击并松开 | 检查机器人、相机和反馈状态后开始一个遥操作训练 episode |
| Back 单击并松开 | 第一次：停止并进入待审核；待审核时再次短按：接受并提交 LeRobot |
| Back 长按 2 秒后松开 | 拒绝当前录制或待审核 episode，不提交 LeRobot，保留原始数据 |
| Back + Start 长按 3 秒 | 仅在控制器报告单一关节限位报警时进入抱闸恢复；松开任意键立即回抱 |
| 松开 deadman | 立即结束 `MoveJog` 或 `ServoP` 控制流 |

`servo_p` 模式会保留六个轴的同时输入，平移向量与旋转向量分别归一化，避免斜推时
合速度超过限值；驱动端再做加速度斜坡，以 45 mm/s、15 deg/s 的适中上限积分目标。
`ServoP` 当前只允许 `JOY_COORD_TYPE=0`。机器人报警、未使能、反馈超时、手柄命令
超过 200 ms 未更新、关节接近软限位、TCP 超出工作空间、松开 LB、急停或节点退出
时，驱动会进入安全保持或停止控制流；通信故障和其它运动模式切换会重建运动连接。
普通移动、MoveJog、拖拽、示教回放和限位恢复与活动的 ServoP 流互斥。

手柄和 ServoP 命令通道只保留最新一条输入，旧摇杆消息不会排队补发。ServoP 目标仍
以 33 Hz 发送，`/cartesian_servo/applied` 以 20 Hz 发布供诊断和 10 Hz 数据采集使用，
不会降低机械臂控制频率。运行日志中的 `Cartesian ServoP timing` 应接近 33 Hz，
`mean_dt` 应接近 30.3 ms；反复出现 `command watchdog` 或明显增大的 `max_dt` 表示
工控机调度或通信仍未按周期运行，应先停止同机的高负载训练和重复相机进程。

Y 和 RB 会先停止当前运动，再调用 `/enable_robot`、`/disable_robot`、`/drag_start`
或 `/drag_stop`，终端日志会打印 accepted/rejected。AG 夹爪 Modbus 手册没有提供运动
急停寄存器，LT/RT 松手停止是通过读取 `0x0202` 实时位置后写入新的 `0x0103` 保持
目标来模拟；若仍有轻微反弹或拖尾，可调整 `JOY_GRIPPER_STOP_LEAD_MM`。若系统没有
`joy_node`，先安装 ROS2 Humble 的 `joy` 包。

A 键夹爪命令不会在上一条命令忙碌时静默丢弃，而是保留最后一次目标并在通道可用时
立即发送。日志中的 `gripper toggle accepted in ... ms` 是从按键到服务返回的总时间，
`gripper_move accepted in ... ms` 是驱动写入末端寄存器的时间，可用来区分手柄调度延迟
和夹爪 Modbus 通信延迟。

夹爪首次从未夹持变为 `object_detected=true` 或 `grip_state=2` 时，teleop 会向 `/joy/set_feedback` 发布一条 `sensor_msgs/msg/JoyFeedback` 短震动。夹爪灯变绿但没有震动时检查：

```bash
ros2 topic info /joy/set_feedback --verbose
ros2 topic echo /joy/teleop_diagnostics
```

`/joy/set_feedback` 应存在 `joy_node` 的兼容订阅者，诊断中的 `subscriber_count` 应大于 0。如果消息能够送达但手柄仍不震动，通常是手柄、内核驱动、设备权限或容器设备映射不支持 force feedback；这不会影响机械臂和夹爪控制。

### 手柄遥操作数据采集

`make joy` 默认同时启动数据采集节点。这是操作员用手柄控制机械臂时的训练数据采集，不是拖拽示教，不调用 `StartDrag()`，也不生成可供 `teach-replay` 直接回放的轨迹。正式训练数据使用 Hugging Face LeRobot Dataset v3.0。

首次部署先执行一次：

```bash
make lerobot-setup
make build
```

LeRobot 当前工具需要 Python 3.12，而 ROS2 Humble 节点使用 Python 3.10，因此安装在仓库的 `.venv-lerobot` 隔离环境中，不修改 ROS Python。安装版本固定到 Makefile 的 `LEROBOT_COMMIT`，避免采集中途因上游 schema 变化导致数据集混用。

每个任务开始前必须设置准确、稳定的任务描述：

```bash
make data-task TASK:="pick up the tape roll"
```

不带 `TASK` 执行 `make data-task` 可以查看当前任务。任务保存在 `data/collections/move_jog/current_task.txt`，采集节点在每次按 Start 时重新读取，因此换任务不需要重启 `make joy`；已经开始的 episode 不会被中途改任务。任务为空时 Start 会拒绝采集，避免生成没有语言条件的训练 episode。

ServoP 采集建议以固定起点运行：先执行 `make servo-data-task TASK:="pick up the tape roll"`，再使用 `make servo-collect` 启动。它等价于以 ServoP 和强制起始位校验启动 `make system`，并在 `data/collections/servo_p_v2` 创建独立的原始数据目录、LeRobot 数据集和起始位文件，防止混入旧的 MoveJog 数据。起始位由操作者确定：短按 RB 进入拖拽，手动移动到安全、张开夹爪的标准姿态，再短按 RB 退出拖拽；随后长按 X 1.5 秒后松开，服务只把当前六轴关节角、TCP、夹爪开度和保存时间写入 `data/collections/servo_p_v2/servo_p_start_pose.json`，不发送运动指令。`make data-set-start` 保留为相同服务的终端调试入口。之后每条 episode 前长按 Start 1.5 秒后松开，或执行 `make data-prepare`，它才会显式 MoveJ 回到该姿态、恢复保存的夹爪开度并校验关节容差；拖拽未退出时会拒绝回位。完成后再短按 Start。未执行 prepare、机器人离开容差或仍在拖拽模式都会拒绝 Start，避免把复位过程和不同起点写入训练数据。第一次 Back 只结束录制并进入审核，机械臂保持当前位置；审核时无论接受还是拒绝，最终都会自动打开夹爪并 MoveJ 回保存的起点，回位过程和结果写入 `events.jsonl`。

运行 `make joy` 并使能机器人后，单独按下再松开 Start 开始采集；完成一次操作后短按 Back。第一次 Back 等待原始图像队列清空并进入 `pending` 待审核状态，同时立即自动打开夹爪并 MoveJ 回保存的起点。确认质量合格后再次短按 Back，系统只把 LeRobot 转换任务放入后台队列并返回，不再重复回位；Qt 面板和 `make data-status` 可观察 `return_phase`、`export_phase`、当前 episode 和队列深度，后台完成后 metadata 才变为 `accepted`。质量不合格时长按 Back 2 秒，系统标记 `rejected`，保留原始数据但不进入 LeRobot，机械臂也不会再次移动。不要同时按 Start 和 Back；该组合键保留给限位恢复。

待审核期间再次按 Start 会被拒绝，避免跳过质量确认。采集节点重启后会自动恢复最近的 pending episode，并重新排队 metadata 标记为 `export_queued` 的任务；也可以使用 `make data-accept` 或 `make data-reject` 完成决定。`make data-status` 的 `phase` 会显示 `recording`、`pending`、`exporting` 或 `idle`。

ServoP 采集启动时默认打开只读 Qt 操作面板（普通 `make system` 默认关闭）。面板显示机器人使能/模式、采集阶段和样本数、自动回位阶段、LeRobot 导出队列、ServoP 实际速度、夹爪状态以及原始手柄 axes/buttons，并固定显示按键说明；面板不发送运动命令。

Qt 面板还会以时间为 X 轴绘制六个关节角度和 Action 值；LeRobot 状态用绿色（成功）、橙色（排队/运行）、红色（失败）表示，手柄操作用按键状态块显示。

Start 会检查 LeRobot v3 环境、任务文本、机器人连接/反馈/使能/报警状态，以及腕部/全局 Image、两份 CameraInfo、同步图像对、关节、TCP、夹爪、Joy 和 teleop action 的新鲜度。任一条件不满足都会拒绝开始并在终端打印原因。

每次采集先生成可恢复、可排障的原始 sidecar：

```text
data/collections/move_jog/episode_YYYYMMDD_HHMMSS/
├── metadata.json
├── camera_info/
│   ├── wrist.json
│   └── global.json
├── steps.jsonl
├── events.jsonl
└── images/
    ├── wrist/frame_000001.jpg ...
    └── global/frame_000001.jpg ...
```

目录内容：

| 文件 | 内容 |
|---|---|
| `metadata.json` | schema 版本、任务文本、单位、Topic、样本数、停止原因和完整性 |
| `camera_info/*.json` | 腕部和全局彩色相机的内参及畸变参数 |
| `steps.jsonl` | 每行一个由同步图像对触发的 observation/action step |
| `events.jsonl` | episode 开始、停止时间和停止原因 |
| `images/wrist/*.jpg` | 腕部相机彩色图像 |
| `images/global/*.jpg` | 全局 RealSense 彩色图像 |

每个 step 包含两张图的路径、时间戳、frame ID 和 `image_pair_skew_sec`，以及关节位置/速度/力矩、TCP、夹爪、机器人状态、原始 Joy 输入和 teleop 实际提交动作。两种不同品牌相机没有硬件同步，采集节点使用 ROS 消息时间戳做近似同步，默认只接受相差不超过 50 ms 的图像对。机器人状态采用该图像对到达时收到的最新值。

采样使用固定时间槽，不要求所有 episode 具有固定时长。10 Hz episode 的 `frame_slot/t` 必须是 `0/0.0, 1/0.1, 2/0.2...`，采样时刻在图像同步回调中确定，后台 JPEG 写盘耗时不会改变时间戳。相机应以高于 10 Hz 的频率发布，由采集器选择每个时间槽附近的同步图像对；若真的漏掉时间槽，`missed_sample_slots` 会增加，该原始 episode 会保留但不会导入 LeRobot，避免把不连续的真实过程压缩成伪 10 Hz 视频。

主动作顺序固定为 `[X,Y,Z,Rx,Ry,Rz]`。`move_jog` 模式记录 `-1/0/1` 的固定速率
方向；`servo_p` 模式记录驱动经过死区、响应曲线、限幅和加速度斜坡后实际应用的
连续归一化速度，允许多个分量同时非零。两种模式 action 语义不同，不可追加到同一个
LeRobot 数据集；切换到 `servo_p` 后应使用新的 dataset root 和 repo id。图像通过
有界后台队列成对写入；Back 会等待队列清空后再关闭文件。

首次低速验收先关闭采集，避免测试动作进入正式数据集：

```bash
JOY_CONTROL_MODE=servo_p JOY_LEROBOT_ENABLED=false make system
```

实机确认斜线、组合运动、松开 LB 和 B 急停都符合预期后，再使用新的训练集启动：

```bash
JOY_CONTROL_MODE=servo_p \
JOY_LEROBOT_DATASET_ROOT="$(pwd)/data/collections/move_jog/lerobot_tape_pi05_v2" \
JOY_LEROBOT_REPO_ID="local/dobot_nova2_tape_pi05_v2" \
make system
```

原始 sidecar 使用本仓库 `format_version=2`，用于断电恢复、相机内参和 ROS 诊断，不是训练入口。只有第二次短按 Back 接受后，同一 episode 才会追加到以下 LeRobot Dataset v3.0 数据集：

```text
data/collections/move_jog/lerobot_tape_pi05/
├── meta/
│   ├── info.json
│   ├── stats.json
│   ├── tasks.parquet
│   └── episodes/chunk-000/file-000.parquet
├── data/chunk-000/file-000.parquet
└── videos/
    ├── observation.images.wrist/chunk-000/file-000.mp4
    └── observation.images.global/chunk-000/file-000.mp4
```

训练字段采用 LeRobot 约定：`observation.state` 为 6 关节弧度、6D TCP 和夹爪开度；`action` 为 `[X,Y,Z,Rx,Ry,Rz]` 固定速率方向及夹爪归一化目标；图像键为 `observation.images.wrist` 和 `observation.images.global`；任务文本由 LeRobot 映射为 `task_index`。`timestamp`、`frame_index`、`episode_index` 和全局 `index` 由官方 API 生成。夹爪 action 在夹爪已初始化且停止运动的首次有效反馈到达后才开始发布，其初始目标与实际开度一致；此后每帧持续记录最近一次有效开合目标，避免把回零过程或录制前的打开状态误记为闭合命令。

该结构依据官方 [LeRobot Dataset v3 文档](https://huggingface.co/docs/lerobot/lerobot-dataset-v3) 和 [LeRobot 官方仓库](https://github.com/huggingface/lerobot)。v3 使用聚合 Parquet/MP4 shard，不要按旧 v2.1 的“每个 episode 一个文件”规则手工移动文件。

`JOY_LEROBOT_DATASET_ROOT` 是上述文件在本机的实际目录；`JOY_LEROBOT_REPO_ID` 是传给官方 `LeRobotDataset.create/resume/load` 的逻辑身份，采用 `所有者/数据集` 格式。默认 `local/dobot_nova2_tape_pi05` 不会自动联网或上传；准备推送 Hugging Face Hub 时再改为实际用户或组织名，例如 `dzkrobot/dobot_nova2_tape_pi05`。

采集中可运行 `make data-status` 检查两路数据新鲜度、当前同步差、时间槽误差、队列、样本数、`missed_sample_slots`、`dropped_pairs` 和 `write_errors`。第一次 Back 后，原始数据完整时 `metadata.json` 标记 `complete=true`、`curation_status=pending`；接受后变为 `accepted`，拒绝后变为 `rejected`。机器人异常、任一路数据超时、节点退出、没有样本、漏槽、队列溢出或写盘错误都会保留目录并标记 `complete=false` 或 `curation_status=incomplete`。

每个 episode 完成后可检查原始 sidecar：

```bash
make data-validate EPISODE:=data/collections/move_jog/episode_YYYYMMDD_HHMMSS
```

检查正式 LeRobot 数据集：

```bash
make data-lerobot-validate
```

该命令使用官方 `LeRobotDataset` 加载数据集并读取第一帧，输出 episode/frame 数和 feature keys。`make data-start` / `make data-stop` 仅用于 service 调试，日常采集使用手柄 Start/Back。

### 手柄限位恢复

限位恢复只用于控制器报告的单一关节限位报警 `64..75`，不能用于其他报警。操作前停止采集，撤离工作空间内人员，清除周边障碍，并由能够承托相关连杆重量的人员操作；抱闸释放后关节可能因重力移动。

1. 保持急停可立即触达，不要按 Y 使能或 LB 点动。
2. 同时按住 Back+Start，持续 3 秒，等待终端确认唯一报警关节并释放抱闸。
3. 继续保持两个按键，由人员缓慢把该关节移出限位。
4. 松开 Back 或 Start 中任意一个键，系统立即重新抱闸。
5. 检查终端结果和 `make errors`；恢复完成后机器人保持 disabled，确认姿态、线缆和人员安全再重新使能。

恢复过程会停止 jog，并拒绝 enable、drag、jog、普通移动和示教回放。手柄侧 10 秒、驱动侧 12 秒分别执行超时回抱；手柄消息中断、B 急停、节点退出或任意组合键松开也会请求回抱。若报警不能唯一对应一个关节，手柄恢复会拒绝执行，应改用前述 `make recover-limit` 交互流程排查，不能猜测关节编号。

## 常用 Topic

| Topic | 类型 | 作用 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节角，供 TF/RViz 和下游节点订阅 |
| `/tcp_pose` | `std_msgs/msg/Float64MultiArray` | TCP 位姿 `[x,y,z,rx,ry,rz]` |
| `/dobot_state` | `dobot_interfaces/msg/DobotState` | 机器人模式、使能、运行、报警等状态 |
| `/gripper_state` | `dobot_interfaces/msg/GripperStatus` | 夹爪初始化、夹持、开口和是否夹住物体 |
| `/keyboard/input` | `std_msgs/msg/String` | 键盘输入事件 |
| `/joy` | `sensor_msgs/msg/Joy` | 手柄输入事件 |
| `/joy/teleop_action` | `dobot_interfaces/msg/TeleopAction` | 实际遥操作动作，供训练数据采集 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 腕部 Orbbec 彩色图像 |
| `/global_camera/color/image_raw` | `sensor_msgs/msg/Image` | 全局 RealSense 彩色图像 |

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

因此工控机上按默认目录放置并构建官方驱动后，可用 `make camera-wrist` 单独启动腕部相机。`make camera` 会同时启动腕部相机和 `src/realsense-ros` 中的全局 RealSense。若官方 Orbbec workspace 放在其它位置，可用 `ORBBEC_WS:=...` 覆盖。

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

标定结果文件保存的是 `Link6 -> camera_color_optical_frame`。启动时工具会结合 Orbbec 官方相机内部 TF，默认换算并发布 `Link6 -> camera_link`，避免 `camera_color_optical_frame` 同时有两个 parent。

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
| `keyboard.translation_step_mm` / `keyboard.rotation_step_deg` | `5.0` / `2.0` |
| `keyboard.motion_service` | `movep` |
| `keyboard.workspace_min` / `keyboard.workspace_max` | `[-625, -625, 20, -360, -360, -360]` / `[625, 625, 625, 360, 360, 360]` |
| `keyboard.workspace_max_xy_radius_mm` | `625.0` |
| `keyboard.gripper_opening_open_mm` / `keyboard.gripper_opening_close_mm` | `95.0` / `0.0` |

`joint_limit_check` 默认开启。`movej` 会在下发前检查目标关节角，`movel`/`movejp`/`movep` 会检查 IK 解出的关节角。若现场控制器配置了更小的软件限位、使用 Nova 5、或工具负载发生变化，需要同步修改这个 YAML。

AG-160-95-W-S 夹爪通过机械臂末端 RS485 接入，默认使用 Dobot 控制器的 Modbus-RTU 转发控制。位置命令支持 `GRIPPER_OPENING_MM`，也支持 `GRIPPER_POS` 千分比；夹持力支持 `GRIPPER_FORCE` 百分比或 `GRIPPER_FORCE_N` 牛顿值，最终会映射到厂家 20-100% 力值寄存器。夹持状态 `2` 表示夹住物体，`3` 表示物体掉落。若改为 USB-RS485 直连电脑调试，可将 `gripper_transport` 改为 `local_serial` 并配置 `gripper_port`。

## 包结构

| 包 | 内容 |
|---|---|
| `dobot_interfaces` | 自定义 msg/srv（DobotState / GripperStatus / MoveCommand 等） |
| `dobot_description` | URDF 模型和 STL mesh |
| `dobot_ros2` | 驱动节点、launch 文件、RViz 配置 |
| `dobot_handeye` | 可选手眼标定工具、在线 board TF 验证 |
| `dobot_camera` | 腕部 Orbbec、全局 RealSense 和双画面查看的 launch 封装 |
| `dobot_keyboard` | 可选键盘输入和笛卡尔小步 teleop |
| `dobot_joy` | 可选手柄输入和 `MoveJog` 连续点动 teleop |

# Dobot Nova2 ROS2 Driver

Dobot Nova2 的 ROS 2 Humble 驱动，以及当前 ServoP 数据采集、OpenPI Docker 训练和真机部署入口。
机械臂相关命令都从本仓库执行，OpenPI 是外部训练/推理服务。

## 1. 准备环境

默认环境：ROS 2 Humble、Orbbec workspace `~/orbbec_305`、Dobot 工作区和两台相机。

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make build
make lerobot-setup       # 首次采集执行一次
```

基础检查：

```bash
make state
make errors
make gripper-state
make camera-check
```

## 2. ServoP 采集配置

采集、训练和部署参数集中在 [config/pi05_pipeline.env](config/pi05_pipeline.env)，包括：

- `SERVO_DATASET_ROOT`：原始 episode 和任务文件目录；
- `SERVO_LEROBOT_DATASET_ROOT`：LeRobot v3 输出目录；
- `SERVO_START_POSE_FILE`：固定起始位；
- `SERVO_TASK`：任务文本；训练 config、实验名、步数、checkpoint 和起始位也在同一文件。

新任务只需要编辑这个文件中的采集段，例如：

```dotenv
SERVO_DATASET_ROOT=/home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver/data/collections/servo_p_pen_box
SERVO_LEROBOT_DATASET_ROOT=/home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver/data/collections/servo_p_pen_box/lerobot_pi05_pen_box
SERVO_START_POSE_FILE=/home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver/data/collections/servo_p_pen_box/servo_p_start_pose.json
SERVO_TASK=pick the pen and put it in the box
```

目录结构保持为：

```text
data/collections/servo_p_pen_box/
├── episode_YYYYMMDD_HHMMSS/       原始 sidecar
├── lerobot_pi05_pen_box/          LeRobot v3 数据集
├── servo_p_start_pose.json        固定起始位
└── current_task.txt               任务文本
```

## 3. 手柄采集流程

### 启动

```bash
make servo-data-task
make servo-collect
```

`servo-collect` 会启动 Dobot、双相机、ServoP 手柄控制、采集节点和 Qt 操作面板。不要同时运行
`make joy`、`make camera` 或其它运动节点。

### 保存起始位

新目录第一次使用时：

1. `Y` 使能机器人，确认夹爪张开。
2. 按 `RB` 进入拖拽，把机械臂放到安全、可重复的标准姿态。
3. 再按 `RB` 退出拖拽。
4. 长按 `X` 1.5 秒后松开，保存起始位。
5. 长按 `Start` 1.5 秒后松开，回到并校验起始位。

### 每条 episode

| 操作 | 功能 |
|---|---|
| `Start` 短按 | 开始录制 |
| `Start` 长按 | 回到起始位 |
| `LB` | deadman，运动时保持按下 |
| `LT` / `RT` / `A` | 夹爪关闭/打开/切换 |
| `Back` 第一次短按 | 结束录制，打开夹爪并回起始位，进入审核 |
| `Back` 第二次短按 | 接受并异步转换为 LeRobot |
| `Back` 长按 2 秒 | 拒绝，保留原始数据 |
| `B` | 急停 |
| `X` 短按 | 清报警 |

Qt 面板显示关节曲线、action、夹爪状态、当前 episode 和 LeRobot 转换队列。绿色表示成功，橙色
表示排队/运行，红色表示失败。转换在后台执行，`Back` 不等待视频编码。

采集状态和数据验收：

```bash
make data-status
make servo-data-lerobot-validate
```

训练前必须确认 LeRobot v3.0、10 FPS、state 13 维、action 7 维、双相机存在，且夹爪 action 的
时间顺序正确：起始打开，任务阶段关闭，结束阶段按任务要求打开。

## 4. 一键 Docker 训练

训练参数集中在 [config/pi05_pipeline.env](config/pi05_pipeline.env)，README 不重复维护参数。
需要修改时只编辑：

- `OPENPI_DOBOT_DATASET_DIR`：LeRobot 数据集路径；
- `OPENPI_POLICY_CONFIG`：OpenPI 注册配置；
- `OPENPI_EXP_NAME`：实验名；
- `OPENPI_TRAIN_STEPS`：训练步数；
- `OPENPI_TRAIN_RESUME` / `OPENPI_TRAIN_OVERWRITE`：续训或新实验开关。

启动训练：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make policy-train
```

训练脚本会自动启动 GPU 容器、计算 normalization stats，并在后台启动训练。输出位置为：

```text
/home/ps/openpi-docker-data/checkpoints/<config>/<experiment>/
├── metrics.csv
├── training_curves.png
├── <step>/params
├── <step>/train_state
└── <step>/assets
```

文本日志：

```text
/home/ps/openpi-docker-data/wandb/<experiment>.log
```

训练监控：

```bash
tail -f /home/ps/openpi-docker-data/wandb/<experiment>.log
watch -n 10 'tail -n 1 /home/ps/openpi-docker-data/checkpoints/<config>/<experiment>/metrics.csv'
watch -n 5 nvidia-smi
```

停止训练：

```bash
cd /home/ps/DZK_repos/openpi
docker compose -f compose.yaml -f compose.gpu.yaml -f compose.dobot.yaml \
  exec openpi pkill -f scripts/train.py
```

继续训练时把 `OPENPI_TRAIN_RESUME=true`、`OPENPI_TRAIN_OVERWRITE=false` 写入配置后再次执行
`make policy-train`。

## 5. 一键部署 checkpoint

部署参数也集中在 [config/pi05_pipeline.env](config/pi05_pipeline.env)，需要修改时只编辑：

- `OPENPI_POLICY_CONFIG`：训练时使用的配置；
- `OPENPI_CHECKPOINT_DIR`：宿主机 checkpoint；
- `OPENPI_CHECKPOINT_CONTAINER_DIR`：同一 checkpoint 在 Docker 中的路径；
- `POLICY_START_POSE_FILE`：与数据集匹配的 ServoP 起始位。

首次更换 checkpoint：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make policy-dry-run
```

确认 Policy Server、双相机、state 和 `(16, 7)` action 正常后，启动真机：

```bash
make policy-real
```

启动流程会等待 Policy Server 健康、初始化夹爪、打开夹爪、MoveJ 回起始位，然后进入暖会话：

| 按键 | 功能 |
|---|---|
| `r` | 停止当前 episode，回起始位并重新运行 |
| `q` / `Ctrl-C` | 停止 policy、停止运动并退出 |

首次实机运行必须握住物理急停，清空工作空间，不要并行运行手柄或其它运动节点。

## 6. 日志和排障

系统日志：

```text
logs/run_YYYYMMDD_HHMMSS_PID/
├── launch.log       彩色终端输出
├── manifest.json    启动配置和数据路径
├── events.jsonl     事件
├── health.jsonl     机器人/相机/手柄状态
└── nodes/           按 ROS 节点拆分
```

Policy 日志：

```text
logs/policy/
├── pi05_tape_grasp_*.jsonl              完整结构化事件
├── pi05_tape_grasp_*.log                可读事件摘要
└── pi05_tape_grasp_*_artifacts/         图像、state、action
```

```bash
make logs-latest
make policy-status
find logs/policy -maxdepth 1 -type f -printf '%T@ %p\n' | sort -n | tail
```

`log/` 是 colcon 构建日志，`logs/` 是运行日志，二者保持分开。异常时优先查看最新 policy JSONL、
同名 `.log`、artifacts 和 OpenPI deploy log。

## 7. 基础安全命令

```bash
make errors
make clear
make enable
make disable
make estop
make recover-limit
```

实机测试始终保留物理急停；报警、相机掉线、夹爪未初始化或 Policy Server 不健康时不要启动
`make policy-real`。

## 相关文档

- [OpenPI Docker 训练记录](../../openpi/docs/dobot_pi05_docker_training.md)
- [pi0.5 实机部署记录](docs/pi05_policy_deployment.md)
- [手柄/采集开发记录](docs/joystick_teleop_development.md)

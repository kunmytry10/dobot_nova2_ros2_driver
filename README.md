# Dobot Nova2 ROS2 Driver

Dobot Nova2 的 ROS 2 Humble 驱动，以及当前 ServoP 数据采集、OpenPI Docker 训练和真机部署入口。
机械臂相关命令都从本仓库执行，OpenPI 是外部训练/推理服务。

## 效果展示
<img width="400" height="240" alt="training" src="https://github.com/user-attachments/assets/345b95a4-ecf0-4b44-ac7e-1d13d2594f7c" /> <img width="400" height="240" alt="1" src="https://github.com/user-attachments/assets/e0813056-f104-42fb-84c0-5af5e946d313" />


## 1. 环境准备

默认环境：Ubuntu 22.04、ROS 2 Humble、NVIDIA GPU 驱动、Docker Engine、Docker Compose
plugin、NVIDIA Container Toolkit、Dobot 工作区和两台相机。

### 外部仓库和驱动

本仓库只包含 Dobot 驱动、采集和调用入口。完整链路还依赖以下外部项目：


| 依赖                  | 上游来源                                                                           | 默认位置                          | 用途                                        |
| --------------------- | ---------------------------------------------------------------------------------- | --------------------------------- | ------------------------------------------- |
| OpenPI Docker 环境    | [QingTianRobot/openpi](https://github.com/QingTianRobot/openpi)                    | `/home/ps/DZK_repos/openpi`       | pi0.5 训练、Policy Server 和`openpi-client` |
| Gemini 305 ROS 2 驱动 | [Yahboom Gemini 305 配套资料](https://www.yahboom.com/build.html?id=17247&cid=747) | `~/orbbec_305/src/OrbbecSDK_ROS2` | 腕部 Gemini 305 相机，提供`orbbec_camera`   |
| RealSense ROS 2 驱动  | [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros)          | `src/realsense-ros`               | 全局 D435 相机，提供`realsense2_camera`     |

RealSense 驱动是本仓库的 Git submodule，当前固定在 4.58.3。首次克隆本仓库时使用
`--recurse-submodules`，已有仓库则执行：

```bash
git submodule update --init --recursive
```

Gemini 305 驱动环境来自上述 Yahboom 配套资料，实际 workspace 中使用
`OrbbecSDK_ROS2` 并提供 `orbbec_camera`。它不在本仓库中，需要在独立的
`~/orbbec_305` workspace 中安装和构建；启动脚本会 source
`~/orbbec_305/install/setup.bash`。如使用其他路径，通过 `ORBBEC_WS` 指定。

OpenPI 也不在本仓库中，需要单独克隆上述 QingTianRobot fork。`dobot-policy-train` 和
`dobot-policy-real` 会读取该仓库的 `compose.yaml`、`compose.gpu.yaml` 和
`compose.dobot.yaml`。默认镜像 `openpi:dev` 由该仓库的 `scripts/docker/dev.Dockerfile`
在本机构建，不是从本 Dobot 仓库提供的预构建镜像。如使用其他路径，通过
`OPENPI_REPO_DIR` 指定。checkpoint、缓存和训练日志默认写入 OpenPI 同级的
`openpi-docker-data/`，不会写入容器临时文件系统。

准备好外部依赖后，再初始化本仓库：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
source functions.zsh    # 自动打印函数用法
dobot-build
# 仅当 .venv-lerobot 缺失、损坏或需要更新固定版本时执行：
dobot-lerobot-setup
```

基础检查：

```bash
dobot-state
dobot-errors
dobot-gripper-state
dobot-camera-check
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

终端 A：

```bash
dobot-servo-task
dobot-servo-collect
```

`dobot-servo-collect` 会持续运行。将机械臂移动到期望的可重复起始位后，在终端 B 执行：

```bash
source /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver/functions.zsh
# Save the desired repeatable start pose once per collection directory.
dobot-data-set-start
# Before each episode, return to and validate that saved pose.
dobot-data-prepare
```

`servo-collect` 会启动 Dobot、双相机、ServoP 手柄控制、采集节点和 Qt 操作面板。不要同时运行
`dobot-system`、`dobot-camera` 或其它运动节点。

### 可视化和控制入口


| 目标                      | 命令                                   | 使用边界                                                                |
| ------------------------- | -------------------------------------- | ----------------------------------------------------------------------- |
| ServoP 采集与 Qt 操作面板 | `dobot-servo-collect`                  | 采集默认入口，Qt 面板已经自动启动。                                     |
| 双相机实时画面            | `dobot-servo-collect SYSTEM_VIEW=true` | 在采集进程中额外打开双相机画面。                                        |
| 机械臂模型和 TF           | `dobot-rviz`                           | 独立排障入口，会启动 driver/robot state publisher；不要与采集入口并行。 |
| 独立控制台                | `dobot-control-ui`                     | 独立排障入口，会启动 driver；不要与采集入口并行。                       |
| 接入已有 driver 的控制台  | `dobot-control-ui-only`                | 仅在已有 driver 已启动时使用；不要用它替代采集 Qt 面板。                |

ServoP 默认启动相机，因此若 `data/handeye/handeye_result.yaml` 存在（或通过 `HANDEYE_STATIC_TF_FILE` 指定其他文件），
系统会同时发布相机 TF。`dobot-tf` 可检查 `/tf` 与 `/tf_static`，`dobot-frames` 可导出完整 TF 树。

### LeRobot 工具环境

`dobot-lerobot-setup` 创建 `.venv-lerobot` 并安装固定版本的 `lerobot[dataset]`。当 episode 被接受后，采集节点会
用该 Python 环境异步执行 LeRobot v3 导出；`dobot-data-validate` 也依赖同一环境。因此不能删除该命令，除非同时关闭
LeRobot 导出并不再需要训练数据。已经存在可用的 `.venv-lerobot` 时无需重复执行它。

### 保存起始位

新目录第一次使用时，`dobot-servo-collect` 已启动后：

1. `Y` 使能机器人，确认夹爪张开。
2. 按 `RB` 进入拖拽，把机械臂放到安全、可重复的标准姿态。
3. 再按 `RB` 退出拖拽。
4. 执行 `dobot-data-set-start` 保存起始位，或长按 `X` 1.5 秒后松开。
5. 每条 episode 前执行 `dobot-data-prepare`，或长按 `Start` 1.5 秒后松开，回到并校验起始位。

若未保存起始位，采集会被拒绝并提示 `prepare rejected`；这是 `COLLECTION_REQUIRE_START_POSE=true` 的预期保护行为。

### 每条 episode


| 操作              | 功能                                   |
| ----------------- | -------------------------------------- |
| `Start` 短按      | 开始录制                               |
| `Start` 长按      | 回到起始位                             |
| `LB`              | deadman，运动时保持按下                |
| `LT` / `RT` / `A` | 夹爪关闭/打开/切换                     |
| `Back` 第一次短按 | 结束录制，打开夹爪并回起始位，进入审核 |
| `Back` 第二次短按 | 接受并异步转换为 LeRobot               |
| `Back` 长按 2 秒  | 拒绝，保留原始数据                     |
| `B`               | 急停                                   |
| `X` 短按          | 清报警                                 |

Qt 面板显示关节曲线、action、夹爪状态、当前 episode 和 LeRobot 转换队列。绿色表示成功，橙色
表示排队/运行，红色表示失败。转换在后台执行，`Back` 不等待视频编码。

采集状态和数据验收：

```bash
dobot-data-status
dobot-data-validate
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
dobot-policy-train
```

训练脚本会自动启动 GPU 容器、计算 normalization stats，并在后台启动训练。输出位置为：

```text
/home/ps/DZK_repos/openpi-docker-data/checkpoints/<config>/<experiment>/
├── metrics.csv
├── training_curves.png
├── <step>/params
├── <step>/train_state
└── <step>/assets
```

文本日志：

```text
/home/ps/DZK_repos/openpi-docker-data/wandb/<experiment>.log
```

训练监控：

```bash
tail -f /home/ps/DZK_repos/openpi-docker-data/wandb/<experiment>.log
watch -n 10 'tail -n 1 /home/ps/DZK_repos/openpi-docker-data/checkpoints/<config>/<experiment>/metrics.csv'
watch -n 5 nvidia-smi
```

停止训练：

```bash
cd /home/ps/DZK_repos/openpi
docker compose -f compose.yaml -f compose.gpu.yaml -f compose.dobot.yaml \
  exec openpi pkill -f scripts/train.py
```

继续训练时把 `OPENPI_TRAIN_RESUME=true`、`OPENPI_TRAIN_OVERWRITE=false` 写入配置后再次执行
`dobot-policy-train`。

## 5. 一键部署 checkpoint

部署参数也集中在 [config/pi05_pipeline.env](config/pi05_pipeline.env)，需要修改时只编辑：

- `OPENPI_POLICY_CONFIG`：训练时使用的配置；
- `OPENPI_CHECKPOINT_DIR`：宿主机 checkpoint；
- `OPENPI_CHECKPOINT_CONTAINER_DIR`：同一 checkpoint 在 Docker 中的路径；
- `POLICY_START_POSE_FILE`：与数据集匹配的 ServoP 起始位。

首次更换 checkpoint：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
dobot-policy-dry-run
```

确认 Policy Server、双相机、state 和 `(16, 7)` action 正常后，启动真机：

```bash
dobot-policy-real
```

启动流程会等待 Policy Server 健康、初始化夹爪、打开夹爪、MoveJ 回起始位，然后进入暖会话：


| 按键           | 功能                                 |
| -------------- | ------------------------------------ |
| `r`            | 停止当前 episode，回起始位并重新运行 |
| `q` / `Ctrl-C` | 停止 policy、停止运动并退出          |

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
dobot-logs-latest
dobot-policy-status
find logs/policy -maxdepth 1 -type f -printf '%T@ %p\n' | sort -n | tail
```

`log/` 是 colcon 构建日志，`logs/` 是运行日志，二者保持分开。异常时优先查看最新 policy JSONL、
同名 `.log`、artifacts 和 OpenPI deploy log。

## 7. 基础安全命令

```bash
dobot-errors
dobot-clear
dobot-enable
dobot-disable
dobot-estop
dobot-recover-limit
```

实机测试始终保留物理急停；报警、相机掉线、夹爪未初始化或 Policy Server 不健康时不要启动
`dobot-policy-real`。

## 8. 补充功能

### 手眼标定

手眼标定实现位于 `src/dobot_handeye`，支持采集、求解、验证、诊断以及静态 TF 发布。标定数据和结果
默认写入被 `.gitignore` 忽略的 `data/handeye/`，不会上传到仓库。

```bash
# 检查相机、TF 和参数是否就绪
dobot-handeye-check

# 采集一组标定姿态（需要先启动机械臂和相机）
dobot-handeye-capture HANDEYE_DATASET_NAME=nova2_camera

# 求解、验证和诊断
dobot-handeye-solve DATASET=nova2_camera HANDEYE_RESULT_FILE=$PWD/data/handeye/handeye_result.yaml
dobot-handeye-validate DATASET=nova2_camera HANDEYE_RESULT_FILE=$PWD/data/handeye/handeye_result.yaml
dobot-handeye-diagnose DATASET=nova2_camera
```

启动 driver、相机或系统时，可通过 `HANDEYE_STATIC_TF_FILE` 指定结果文件；存在有效结果时，系统会
发布 `Link6 -> camera_link` 的静态 TF。`dobot-handeye-tf` 可单独发布该变换，
`dobot-handeye-board-tf` 可发布检测到的标定板 TF。
<img width="800" height="480" alt="手眼" src="https://github.com/user-attachments/assets/5620642a-ceea-4e11-b2d9-d2605d80c556" />


### 键盘驱动

`src/dobot_keyboard` 提供终端键盘控制和 Linux 输入设备控制两种入口。完整键盘控制会启动输入节点和
teleop 节点；Jog 模式会读取 `/dev/input/event*` 并执行受限 jog。

```bash
# 终端键盘步进控制
dobot-keyboard

# Linux 输入设备 Jog（按实际设备修改）
dobot-keyboard-jog KEYBOARD_DEV=/dev/input/event0
```

也可以分别启动 `dobot-keyboard-input`、`dobot-keyboard-jog-input` 和
`dobot-keyboard-teleop`，并通过 `KEYBOARD_STEP_MM`、`KEYBOARD_ROT_STEP_DEG`、`SPEED`、
`ACC` 等变量调整步长和运动参数。键盘控制前确认 driver 已启动、工作空间清空，并保留急停可用。

### 控制 UI

`dobot-control-ui` 启动 ROS driver、robot state publisher 和 Web 控制台；默认监听
`http://localhost:8080`。页面提供机器人状态、关节和 TCP 位姿、夹爪控制、MoveJ/MoveL、示教轨迹和急停
等操作。已有 driver 时使用 `dobot-control-ui-only`，避免重复启动运动节点；端口可通过
`CONSOLE_PORT=8081 dobot-control-ui` 修改。
<img width="800" height="480" alt="ui" src="https://github.com/user-attachments/assets/ba31b54d-ce78-4db8-87dd-7b585668ce67" />

### RViz 与 TF 树

```bash
# 启动模型、robot_state_publisher、RViz 和可用的手眼静态 TF
dobot-rviz

# 查看 TF 话题并生成完整 TF 树图
dobot-tf
dobot-frames
```

`dobot-frames` 生成的 `frames_*.gv` 和 `frames_*.pdf` 仅用于本地排障，已加入忽略规则。需要查看
相机链路时，重点检查 `Link6`、`camera_link` 和 `camera_color_optical_frame` 是否连通，并确认没有
重复运行多个 driver 或静态 TF 发布节点。

<img width="488" height="480" alt="2" src="https://github.com/user-attachments/assets/f623f0b7-e3c6-428d-b907-8ff29b976f36" />

## TF树

[tf.pdf](https://github.com/user-attachments/files/30737152/tf.pdf)


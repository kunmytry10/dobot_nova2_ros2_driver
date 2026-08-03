# Dobot Nova2 pi0.5 实机部署记录

本文独立记录从最终 checkpoint、Docker Policy Server、ROS 2 policy node 到 Dobot Nova2 实机闭环抓取的部署过程。训练过程见 OpenPI 仓库的 `docs/dobot_pi05_docker_training.md`。后续实机联调结果继续追加到本文末尾。

## 当前有效配置（2026-08-03）

以下配置是现在唯一推荐的实机入口。README 中的命令会从这些默认值启动；更换模型时只需
覆盖 `OPENPI_POLICY_CONFIG`、`OPENPI_CHECKPOINT_DIR` 和对应的容器内 checkpoint 路径。

```text
任务：pick the pen and put it in the box
配置：pi05_dobot_pen_box_servo_p_action_only
checkpoint：/home/ps/openpi-docker-data/checkpoints/
  pi05_dobot_pen_box_servo_p_action_only/
  dobot_pen_box_servo_p_action_only_v1_long/135000
控制：ServoP（action-only，7 维，夹爪为打开/关闭二值目标）
起始位：data/collections/servo_p_v2/servo_p_start_pose.json
数据集：data/collections/servo_p_v2/lerobot_pi05_servo_p_v2
```

从 Dobot 仓库启动：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make policy-real
```

脚本默认跳过重复 `colcon build`、复用已经健康的 Policy Server，并进入暖会话。按 `r` 会停止
当前 episode、打开夹爪、回到记录起点并重新运行；按 `q` 或 `Ctrl-C` 退出。默认 episode 上限为
90 秒，可用 `POLICY_MAX_EPISODE_SEC` 覆盖。每次运行的 JSONL、可读文本日志和观测 artifacts
位于 `logs/policy/`，ROS 节点日志位于 `logs/ros_policy_*/`。

> 本节是当前入口。下方旧 tape/MoveJog、ServoP v1 和夹爪标签缺陷内容均为历史记录，不能作为
> 当前 checkpoint 的启动命令。

> 2026-08-01 当前部署已切换到 ServoP 模型。前面的 MoveJog 内容保留为历史记录。

## 1. 部署目标

任务：`pick up the tape roll`

最终 checkpoint：

```text
/home/ps/DZK_repos/openpi-docker-data/checkpoints/
  pi05_dobot_tape_action_only/dobot_tape_action_only_v1/19999
```

部署链路：

```text
/camera/color/image_raw + /global_camera/color/image_raw
                       + /joint_states + /tcp_pose + /gripper_state
                                      |
                                      v
                           dobot_policy_node
                                      |
                    WebSocket, 127.0.0.1:8000
                                      |
                                      v
                 Docker OpenPI Policy Server, checkpoint 19999
                                      |
                              (16, 7) actions
                                      |
                 clamp + one-hot axis safety decoder + watchdog
                                      |
                                      v
                         /move_jog + /gripper_move
                                      |
                                      v
                         Dobot Nova2 -> next observation
```

模型在 GPU Docker 容器中运行。ROS 2 节点不加载 JAX 权重，只安装轻量的 `openpi-client`，负责采集观测、请求推理、过滤 action 和执行机器人命令。

当前 shell 曾同时存在新旧代理变量。`websockets` 16 会自动读取这些变量，可能错误地把本机 `ws://127.0.0.1:8000` 送进代理。一键脚本和 policy node 都会对该本机/LAN policy 连接显式禁用代理；这不会修改 Docker daemon 的镜像下载代理。

## 2. 当前 checkpoint 的动作语义

训练数据 `lerobot_tape_pi05` 的 31 个 episode 来自 `move_jog` 模式。六维运动 action 是：

```text
[X, Y, Z, Rx, Ry, Rz]
```

每帧只有一个轴为 `-1` 或 `1`，其余轴为 `0`。第七维是归一化夹爪目标。因此本 checkpoint 部署到 `/move_jog`，不能直接解释成连续 ServoP 速度。

节点执行以下过滤：

- 验证输出严格为 `(16, 7)` 且没有 NaN/Inf；
- 把训练外的轻微超界值 clamp 到 `[-1, 1]`；
- 最大运动分量必须达到 `0.50`；
- 最大和第二大分量差必须达到 `0.15`，否则视为多轴歧义并停止 jog；
- 本训练集旋转 action 恒为 0，因此当前部署明确禁止 `Rx/Ry/Rz` 命令；
- 训练集的第七维只有离散值 `0`（闭合）和 `1`（张开）。部署端以 `0.5` 为阈值，严格下发 `0 mm` 或 `95 mm`，不会下发中间开度；
- 每段只执行前 4 步，并在剩余 2 步时异步请求下一段。

2026-07-31 新采集的 `servo_p` episode 没有加入本次训练集。后续若要连续、平滑的多轴速度控制，需要积累完整的 ServoP 数据集并重新训练，不应把两种 action 语义混在同一数据集中。

## 3. 新增 ROS 2 包

包路径：

```text
src/dobot_policy/
```

主要文件：

- `dobot_policy/policy_node.py`：闭环节点、WebSocket inference worker、执行状态机和日志；
- `dobot_policy/policy_logic.py`：action 校验、MoveJog 解码、state 与夹爪映射；
- `config/pi05_tape_grasp.yaml`：当前 checkpoint 的部署参数；
- `launch/pi05_tape_grasp.launch.py`：机械臂、双相机和 policy node 的组合 launch；
- `test/test_policy_logic.py`：不连接硬件的纯逻辑测试。

ROS 接口：

```text
/dobot_policy/start   std_srvs/srv/Trigger
/dobot_policy/stop    std_srvs/srv/Trigger
/dobot_policy/status  std_srvs/srv/Trigger
/dobot_policy/diagnostics  std_msgs/msg/String
```

默认配置为 `armed: false`。只有实机 launch 显式传入 `armed:=true` 后，节点才会调用 `/move_jog` 和 `/gripper_move`。

## 4. 安全状态机

armed 模式执行动作前必须同时满足：

- `/dobot_state` 已连接、feedback 有效、机器人已使能且无错误；
- `/joint_states`、`/tcp_pose`、`/gripper_state` 和双相机图像全部存在；
- 所有来源更新时间不超过 1 秒；
- 六个关节均在配置极限内保留至少 5 度余量；
- TCP 必须保持在本任务限定区域 `x[-400,-80] / y[-250,120] / z[150,380] mm`；
- 夹爪已连接、状态读取成功并已初始化；
- Policy Server 输出 shape 和数值检查通过。

任何来源过期、推理超时、WebSocket 错误、非法 action、服务拒绝、机器人错误、关节接近限位或 episode 超时都会清空 action 队列并请求 jog stop。

Policy launch 还会在 Dobot driver 内启用 `0.5 s` MoveJog heartbeat watchdog。Policy 节点在每个 10 Hz 控制周期刷新当前轴命令；即使 policy 进程崩溃或被强制杀死，driver 在失去心跳后也会独立发送 `MoveJog()` 停止命令。普通 joystick/keyboard launch 默认不启用该 watchdog，避免改变原有控制行为。

夹爪报告 `object_detected` 且策略正在请求闭合时，节点记录抓取成功候选。随后继续执行 2 秒 post-grasp action，让模型有机会抬起胶带，再停止动作并报告成功。若期间报告 `object_dropped`，本轮按失败处理。

确认抓取后，节点会锁住当前闭合目标，忽略模型随后可能出现的打开夹爪输出；若夹爪停止运动后不再检测到物体，本轮同样按失败停止，避免释放胶带后误报成功。

一键脚本退出或收到 Ctrl-C 时按顺序请求：

1. `/dobot_policy/stop`；
2. `/disable_robot`；
3. 停止 ROS launch；
4. 停止临时 Policy Server。

物理急停仍是最高优先级保护，实机测试时必须握在操作者手中。

## 5. 环境与构建

一键脚本会自动完成以下工作：

- 创建使用系统 ROS Python 包的 `.venv-policy`；
- 从本机 OpenPI 仓库安装 `packages/openpi-client`；
- `colcon build --symlink-install --packages-up-to dobot_policy`；
- 启动 GPU Docker 容器和 checkpoint 19999 Policy Server；
- 等待 Policy Server 的 `http://127.0.0.1:8000/healthz` 返回成功；Docker 端口映射本身不代表模型已加载并可接受 WebSocket 请求；
- 启动 Dobot driver、双相机和 armed policy node。

单独构建整个工作区仍可使用：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make build
```

## 6. Dry-run 验证

Dry-run 会运行真实相机、状态和模型推理，但不会使能机器人，也不会调用运动或夹爪服务。它在获得首个完整 action chunk 后自动退出：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make policy-dry-run
```

状态与停止命令：

```bash
make policy-status
make policy-stop
```

## 7. 一键实机抓取

> 本节记录旧 MoveJog checkpoint 的历史操作。2026-08-01 的 ServoP v1 已因夹爪标签缺陷
> 在脚本中硬性阻断，当前执行 `make policy-real` 会直接拒绝，不会启动机器人。

执行前：

- 把机械臂放回采集示范时的初始位姿和相同 user/tool 坐标系；
- 相机位置、分辨率、曝光和视野应与采集时一致；
- 胶带放在示范分布覆盖的位置；
- 清空机械臂工作空间，确认夹爪、相机线缆没有干涉；
- 操作者握住物理急停，站在机械臂工作空间外；
- 不要同时运行 joystick、keyboard 或其他运动节点。

一键命令：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make policy-real
```

该命令会显示 5 秒倒计时，然后自动使能机器人并开始闭环抓取。夹爪初始化必须在实机策略前单独确认成功；一键策略不会反复发送 home/初始化命令。成功条件是夹爪检测到物体，并完成配置的 post-grasp 抬升窗口。运行中随时按 Ctrl-C 会触发停止和 disable 清理流程。

## 8. 日志

ROS policy 结构化日志：

```text
/home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver/logs/policy/
  pi05_tape_grasp_YYYYMMDD_HHMMSS.jsonl
  pi05_tape_grasp_YYYYMMDD_HHMMSS.log
```

每条记录包含 inference 延迟、原始 action、安全解码结果、夹爪目标、队列长度、抓取检测和最终成功/失败原因。

`.jsonl` 保留完整、可供脚本解析的事件字段；同名 `.log` 按 `时间 级别 事件 关键字段` 排列，
适合现场直接阅读。终端通过 `RCUTILS_COLORIZED_OUTPUT=1` 对 INFO/WARN/ERROR 使用不同颜色，
不会向 JSONL 或可读日志写入 ANSI 控制字符。

每次运行还会创建同名 `_artifacts` 目录：

```text
pi05_tape_grasp_YYYYMMDD_HHMMSS_artifacts/
  request_0001_global.jpg
  request_0001_wrist.jpg
  request_0001_state.npy
  request_0001_actions.npy
  ...
```

因此一次异常运动可以按 `request_id` 完整复盘：当时两路相机实际看到了什么、13 维状态是什么、模型返回的完整 16×7 action、节点执行了哪一个 MoveJog 轴、夹爪目标、TCP/关节反馈以及驱动服务是否接受命令。JPEG 使用 quality 90；典型 45 秒运行预计产生几十 MB，而不是把未压缩图像直接写进 JSON。

Policy Server 日志：

```text
/home/ps/DZK_repos/openpi-docker-data/wandb/
  dobot_tape_action_only_v1_deploy.log
```

出现异常时无需复制完整终端输出。只需保留当次命令的结束状态，并让我检查最新的 `logs/policy/*.jsonl`、同名 artifacts 目录及上述 Policy Server 日志；这些文件足以还原输入画面、模型 action、保护状态机决策和服务错误。

## 9. 2026-07-31 实现记录

- 确认最终 checkpoint 的离线 WebSocket 链路已通过，暖态模型 inference 约 74 ms；
- 确认训练 action 是 MoveJog one-hot 语义，而不是 ServoP 连续速度；
- 新增独立 `dobot_policy` ROS 2 package；
- 实现 13 维 state 和双相机 observation；
- 实现异步 WebSocket inference 和 4-step receding-horizon 执行；
- 实现 action clamp、阈值、轴歧义过滤、关节余量和 source watchdog；
- 增加只在 policy launch 启用的 driver-side MoveJog heartbeat watchdog；
- 实现夹爪目标映射、抓取检测和 post-grasp 窗口；
- 实现 dry-run、armed、start/stop/status 和 JSONL 日志；
- 新增一键脚本和 `make policy-real` 入口；
- 创建 `.venv-policy` 并验证 `rclpy`、`cv_bridge`、`openpi-client`、NumPy 和 websockets 可以在同一 Python 3.10 环境导入；
- `colcon build --symlink-install --packages-up-to dobot_policy` 成功构建 9 个相关包；
- policy 逻辑、driver watchdog 和原有 driver/launch 回归测试共 `62 passed`，Ruff `E/F` 检查和 `git diff --check` 通过；
- ROS launch 在 `start_robot:=false start_camera:=false armed:=false` 下完成无硬件 smoke test；
- ROS 主机客户端连接 Docker checkpoint 19999 并完成真实帧推理：首次 JIT 往返约 `8107 ms`，暖态往返约 `119.5 ms`，输出 `(16, 7)` 全部有限；
- 同一真实帧的前四步被安全解码为 `Z+ / Z+ / Z+ / stop`，与保存的模型输出一致；
- 验证中发现并修复 host WebSocket 误走 shell proxy 的问题；临时 Policy Server 已停止，GPU 显存回落至约 `1.1 GiB`；
- 尚未在本次代码实现阶段发送任何真机运动命令。实机结果将在首次 armed 验证后追加。
- 首次 armed 验证的首个 observation 和双相机图像已保存到 `logs/policy/pi05_tape_grasp_20260731_104702*`。机器人在模型 action 执行前因 Policy Server 尚处于模型加载阶段而停止；日志记录为 `inference_failed: did not receive a valid HTTP response`，并确认 `MoveJog()` stop 与 `disable_robot` 已成功。
- 一键脚本现改为等待 `/healthz`，避免把 Docker 已发布的 8000 端口误判为 WebSocket Policy Server 就绪；节点关闭时也会跳过无效 ROS context 中的重复 jog-stop 查询，避免将正常停止报告为进程异常。
- 最新实机日志 `pi05_tape_grasp_20260731_105525.jsonl` 显示模型首个夹爪 action 为 `0.001507`，映射目标为 `0.143 mm`（闭合），并非打开。失败原因是 Dobot Modbus 夹爪读写返回 `-1`；一键策略已改为不自动重复初始化，且在夹爪状态读取失败时不会执行 policy action。夹爪通讯恢复前禁止继续 armed 抓取。

## 10. 2026-08-01 ServoP 模型部署

### 10.1 模型和执行链路

当前 ServoP checkpoint 为：

```text
/home/ps/DZK_repos/openpi-docker-data/checkpoints/
  pi05_dobot_tape_servo_p_action_only/dobot_tape_servo_p_action_only_v1/29999
```

Policy Server 使用配置 `pi05_dobot_tape_servo_p_action_only`。ROS policy node 已增加
`control_mode=servo_p`：模型前六维经过 finite/shape 检查和 `[-1,1]` 限幅后，以 10 Hz
发布到 `/cartesian_servo/command`。Dobot driver 在 33 Hz 下把归一化速度积分为 ServoP
目标，并保留 0.2 秒 command watchdog、反馈 watchdog、关节余量和 workspace 保护。

accepted 数据中 Rx/Ry 恒为零，因此部署配置使用 axis scales
`[1,1,1,0,0,1]`，允许 X/Y/Z/Rz 并屏蔽未训练的 Rx/Ry。第七维仍以 0.5 为阈值，
只产生 `0 mm` 关闭或 `95 mm` 打开两种夹爪命令。

### 10.2 已完成验证

- step 29,999 checkpoint 保存完整，Orbax 后台保存线程无错误；
- policy 单元测试 10 项通过，相关 9 个 ROS 2 包构建成功；
- `make policy-dry-run` 使用真实双相机、机器人状态和夹爪状态完成 WebSocket 推理；
- server 确认加载 ServoP checkpoint 29999 及其专用 normalization stats；
- 首次 JIT inference 为约 8.24 秒，暖态 inference 为约 109 ms；
- 输出为有限的 `(16,7)` action，dry-run 未使能机器人、未发布运动、未操作夹爪；
- JSONL、双相机 JPEG、state NPY 和完整 action NPY 均已保存到
  `logs/policy/pi05_tape_grasp_20260801_091515*`。
- 修复 ROS shutdown 路径后再次执行 `make policy-dry-run` 成功，最终日志为
  `logs/policy/pi05_tape_grasp_20260801_093050*`；policy、driver 和相机进程均 cleanly finished，
  没有退出 traceback。

### 10.3 发现的 gripper 标签缺陷

dry-run 的第一个 gripper action 接近 0，会立即请求关闭。复核全部 42 个 accepted 原始
episode 后确认：每个 episode 的物理夹爪都从 95 mm 打开状态开始，并在接近目标后才闭合；
但其中 38 个 episode 的 `action.gripper_target_normalized` 从第 0 帧起就被错误记录为 0。
只有前 4 个 episode 正确记录了“先打开、再关闭”。

根因是采集节点自动打开夹爪并归位后，手柄节点仍保留上一轮的关闭目标。手柄节点现已在
Start 成功时根据真实夹爪反馈重新同步二值目标，后续新采数据不会再继承这个旧目标。
现有 LeRobot 数据和 checkpoint 不会因此自动修复。

为防止误操作，`make policy-real` 当前默认硬性拒绝启动；只有 dry-run 可用。不要通过设置
`POLICY_ALLOW_UNSAFE_SERVO_V1=true` 绕过保护。下一步必须重标/重采这些 gripper action，
重新计算 normalization stats 并训练新的 checkpoint，然后再次执行 dry-run 验收。

### 10.4 当前可用命令

安全部署验证：

```bash
cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
make policy-dry-run
```

该命令自动构建 ROS package、启动 GPU Docker、加载 checkpoint、等待 `/healthz`、启动双相机
和状态链路、完成一次真实 observation 推理，然后安全清理。Policy Server 日志位于：

```text
/home/ps/DZK_repos/openpi-docker-data/wandb/
  dobot_tape_servo_p_action_only_v1_deploy.log
```

修复数据并训练新模型后，仍使用同一个一键入口 `make policy-real`，但必须先更新脚本中的
checkpoint/config 并移除已满足条件的 v1 安全阻断。

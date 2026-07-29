# Joystick Teleop Development Notes

本文记录 Dobot Nova2 手柄控制功能的开发过程、现场问题和当前方案。

## 目标

- 使用 ROS2 `sensor_msgs/msg/Joy` 接收手柄输入。
- 按住 LB 作为 deadman，松开立即停止机械臂点动。
- 左摇杆控制 TCP 在 X/Y 平面移动，右摇杆控制 Z 和 Rz。
- DPad 控制 Rx/Ry。
- LT/RT 控制 AG 夹爪连续关闭/打开，A 键做夹爪开关切换。
- B 键急停，X 键清除报警。
- Y 键切换 enable/disable，RB 键切换纯拖拽模式。
- 机械臂接近关节软限位、报警、未使能或反馈无效时拒绝继续 jog。
- 夹爪夹住物体时尝试通过 `/joy/set_feedback` 触发手柄短震动。
- Start/Back 控制手柄遥操作训练 episode 的开始和保存。
- 腕部 Orbbec 和全局 RealSense 图像按时间戳近似同步后成对写入。
- Back+Start 长按进入带自动回抱保护的关节限位恢复。

## 实现路径

手柄功能放在独立包 `dobot_joy` 中，不放进基础驱动包：

- `joy_node` 负责从 `/dev/input/js0` 读取手柄并发布 `/joy`。
- `dobot_joy_teleop` 订阅 `/joy`，把摇杆/DPad 转换为 `/move_jog` service。
- 机械臂运动使用 Dobot 官方 `MoveJog` 点动接口，开始运动发送轴向命令，回中或释放 deadman 发送停止命令。
- 夹爪控制调用 `/gripper_move` 和 `/get_gripper_state`。

当前启动命令：

```bash
make system
```

`make system` 默认组合完整 bringup、双相机、手柄和数据采集；`ROBOT_MODE=driver` 只启动基础驱动，`ROBOT_MODE=external` 连接外部已有驱动，`SYSTEM_VIEW=true` 可同时打开双相机画面。单模块排障时仍使用 `make bringup`、`make camera`、`make camera-view` 和 `make joy`。

任务描述持久保存在数据目录中，采集节点每次 Start 时重新读取：

```bash
make data-task TASK:="pick up the red block"
```

只启动执行节点，复用已有 `/joy`：

```bash
make joy-teleop
```

## 当前默认键位

| 输入 | 功能 |
|---|---|
| LB | deadman，必须按住才允许机械臂 jog |
| 左摇杆上下 | X 方向 |
| 左摇杆左右 | Y 方向 |
| 右摇杆上下 | Z 方向 |
| 右摇杆左右 | Rz |
| DPad 左右 | Rx |
| DPad 上下 | Ry |
| LT | 夹爪关闭 |
| RT | 夹爪打开 |
| A | 夹爪开/关切换 |
| B | 急停 |
| X | 清除报警 |
| Y | enable/disable 切换 |
| RB | 拖拽模式开启/关闭 |
| Start 单击 | 开始遥操作训练 episode |
| Back 单击 | 停止并保存 episode |
| Back + Start 长按 | 限位恢复，松开任意键重新抱闸 |

如果现场某个方向反了，优先改符号参数，例如：

```bash
make joy JOY_X_AXIS_SIGN:=1.0
```

如果手柄轴编号和当前默认不一致，改索引参数：

```bash
make joy JOY_X_AXIS_INDEX:=1 JOY_Y_AXIS_INDEX:=0 JOY_RX_AXIS_INDEX:=6 JOY_RY_AXIS_INDEX:=7
```

## 遇到的问题和处理

### 容器里没有手柄设备

本机容器中看不到 `/dev/input`，导致 `/dev/input/js0` 不存在，`joy_node` 虽然启动但 `/joy` 没有有效数据。现场先在工控机裸机验证，确认手柄和 ROS2 `joy` 链路正常。后续如果必须在容器里跑，需要启动容器时透传 `/dev/input`，并处理 input 设备权限。

### 缺少 ROS2 joy 包

首次启动 `make joy` 报 `package 'joy' not found`。处理方式是在目标系统安装 Humble 对应包：

```bash
sudo apt install ros-humble-joy
```

### 手柄有约 2 秒延迟

现象是 A 键夹爪响应快，但摇杆控制机械臂明显延迟。通过 `/joy/teleop_diagnostics` 和 `/move_jog/diagnostics` 发现 teleop 在空闲时以 50 Hz 重复发送 `MoveJog()` 停止命令，服务调用被 stop 请求淹没。

修复方式：

- 只有存在正在运行的 `current_axis` 时才发送 stop。
- deadman 松开、摇杆回中、节点退出和异常状态仍然会主动 stop。
- `joy_node` 使用 `autorepeat_rate=50.0` 保持摇杆状态刷新，但不再制造空闲 stop 风暴。

### 方向和轴映射调整

现场反馈左右、上下和 Rz 方向不符合操作习惯后，先通过 Makefile 参数调整方向符号。之后确认需要交换的是 DPad X/Y，不是左摇杆 X/Y，因此当前状态为：

- 左摇杆恢复为 `X=axis1, Y=axis0`。
- DPad 改为 `Rx=axis6, Ry=axis7`。

### LT/RT 夹爪松手反弹

AG 夹爪手册里，基础 RS485 控制主要提供：

- `0x0100` 初始化。
- `0x0101` 力值。
- `0x0103` 目标位置。
- `0x0201` 夹持状态反馈。
- `0x0202` 实时位置反馈。

手册中的 `0x0304 停止位` 是 Modbus 通讯停止位配置，不是夹爪运动停止命令。因此夹爪没有类似机械臂 `MoveJog()` 的运动 stop service。

当前处理方式是模拟点动：

- LT 按下时发送闭合端点目标。
- RT 按下时发送张开端点目标。
- 松开时主动调用 `/get_gripper_state` 读取实时开口。
- 根据原运动方向加一个很小的前置补偿，再发送新的保持目标，减少因为“读反馈到写目标”延迟造成的反向回弹。

补偿参数是 `JOY_GRIPPER_STOP_LEAD_MM`，默认 `3.0`。如果仍有反弹，可以增大；如果松手后拖尾太明显，可以减小：

```bash
make joy JOY_GRIPPER_STOP_LEAD_MM:=1.0
make joy JOY_GRIPPER_STOP_LEAD_MM:=5.0
```

这不是硬件级精准刹停。要做到真正“松手即停在释放瞬间的位置”，需要厂商提供运动停止寄存器或支持速度/力控式连续控制接口。

### enable/disable 和拖拽模式

手柄 Y 键会先停止当前 `MoveJog()`，再根据 `/dobot_state.enable_status` 在 `/enable_robot` 和 `/disable_robot` 之间切换。机器人处于 error 状态时拒绝 enable，避免在报警未处理时重新上电。

RB 键调用纯 Dashboard 拖拽服务 `/drag_start` 和 `/drag_stop`。它只对应 `StartDrag()` / `StopDrag()`，不走 `/teach_start`，因此不会开始录制轨迹。需要录轨迹时仍然使用 `make teach-start` 和 `make teach-stop`。

### 震动消息类型

ROS2 Humble `joy_node` 在 `/joy/set_feedback` 订阅的是单条 `sensor_msgs/msg/JoyFeedback`。早期实现发布 `JoyFeedbackArray`，Topic 名相同但类型不兼容，夹爪检测到物体后消息无法到达 `joy_node`。当前已改为 `JoyFeedback`，并在诊断 Topic 中记录震动强度和兼容订阅者数量。

### 遥操作训练数据

这里的数据采集不是拖拽示教，不调用 `StartDrag()`，也不生成用于轨迹回放的示教文件。采集节点按腕部/全局相机同步图像对生成 observation/action step：

- observation：腕部和全局彩色图像、关节位置/速度/力矩、TCP、机器人和夹爪状态。
- action：teleop 实际提交的 `[X,Y,Z,Rx,Ry,Rz]` 固定速率方向和夹爪目标。
- debug：原始 Joy axes/buttons 和离散事件。

两种相机没有硬件同步，ROS 时间戳近似同步容差默认 50 ms。Start 前会检查两路 Image、两份 CameraInfo、同步图像对、其他数据源新鲜度和机器人状态。数据通过有界后台队列成对写盘，Back 等待队列清空后结束。

采集器按固定 FPS 时间槽记录：10 Hz 时 `frame_slot/t` 为 `0/0.0, 1/0.1...`。时间戳在同步图像回调中确定，不使用后台写盘完成时间。相机发布频率应高于采样频率；若同步源未覆盖某个时间槽，记录 `missed_sample_slots` 并将 episode 标记为不完整，禁止导入 LeRobot。只有存在样本且没有漏槽、队列丢帧或写入错误时，`metadata.json` 才标记 `complete=true`。

统一启动每次创建 `logs/run_*/`，`manifest.json` 记录版本、任务和模式，`events.jsonl` 记录状态变化和 teleop/采集结果，`health.jsonl` 记录每秒就绪状态及相机实际帧率，`launch.log` 和 `ros/` 保存完整进程日志。

双相机原始 sidecar 使用 `format_version=2`，分别保存到 `images/wrist`、`images/global` 和 `camera_info/wrist.json`、`camera_info/global.json`。第一次 Back 后进入 pending 审核，第二次短按 Back 才通过隔离的 Python 3.12 环境和官方 LeRobot API 追加到 Dataset v3.0；长按 Back 标记 rejected，不进入训练集。数据包含 `observation.state`、`action`、`observation.images.wrist`、`observation.images.global` 和 task。使用 `make data-validate EPISODE:=...` 检查原始数据，使用 `make data-lerobot-validate` 通过官方加载器检查训练数据集。

### 手柄限位恢复

Back+Start 必须持续按住 3 秒。驱动只接受控制器报警表中的 `64..75`，且必须唯一识别出一个限位关节。流程会停止 jog、禁用机器人、清报警并检查模式，然后才释放对应关节抱闸。松开任意键立即重新抱闸；手柄侧默认 10 秒、驱动侧默认 12 秒还有独立超时保护。恢复过程中驱动拒绝 enable、drag、jog、普通运动和示教回放，恢复结束后仍保持 disabled。

## 验证命令

启动前确认手柄 topic：

```bash
ros2 topic list | grep joy
ros2 topic hz /joy
ros2 topic echo /joy --once
```

观察手柄 teleop 到运动服务的耗时：

```bash
ros2 topic echo /joy/teleop_diagnostics
ros2 topic echo /move_jog/diagnostics
```

启动控制：

```bash
make camera
make joy
```

安全验证顺序：

1. 先不按 LB，确认摇杆不会移动机械臂。
2. 按住 LB，小幅推动摇杆，确认按下动、回中停。
3. 测试 B 急停和 X 清报警。
4. 测试 DPad 左右/上下旋转方向。
5. 空夹状态下测试 LT/RT，确认松开不会明显反弹。
6. 夹取软物体测试震动反馈和 `/gripper_state`。

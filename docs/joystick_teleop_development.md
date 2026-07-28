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

## 实现路径

手柄功能放在独立包 `dobot_joy` 中，不放进基础驱动包：

- `joy_node` 负责从 `/dev/input/js0` 读取手柄并发布 `/joy`。
- `dobot_joy_teleop` 订阅 `/joy`，把摇杆/DPad 转换为 `/move_jog` service。
- 机械臂运动使用 Dobot 官方 `MoveJog` 点动接口，开始运动发送轴向命令，回中或释放 deadman 发送停止命令。
- 夹爪控制调用 `/gripper_move` 和 `/get_gripper_state`。

当前启动命令：

```bash
make joy JOY_DEV:=/dev/input/js0
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
| Back / Start | 停止当前点动 |

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
make joy JOY_DEV:=/dev/input/js0
```

安全验证顺序：

1. 先不按 LB，确认摇杆不会移动机械臂。
2. 按住 LB，小幅推动摇杆，确认按下动、回中停。
3. 测试 B 急停和 X 清报警。
4. 测试 DPad 左右/上下旋转方向。
5. 空夹状态下测试 LT/RT，确认松开不会明显反弹。
6. 夹取软物体测试震动反馈和 `/gripper_state`。

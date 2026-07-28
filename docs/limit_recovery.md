# Dobot Limit Recovery

本文记录关节限位抱死后的推荐恢复流程。

## 背景

当机械臂关节触发正/负限位后，机器人可能进入 error 或 brake 状态，普通 `make clear`、`make enable`、示教拖拽都无法恢复。现场曾出现关节 6 负向限位，`GetErrorID()` 返回 `75: 关节6负向限位`。

## 一键向导

使用：

```bash
make recover-limit
```

如果无法自动识别关节，手动指定：

```bash
make recover-limit JOINT:=6
```

## 向导会做什么

1. 连接 Dashboard `192.168.5.1:29999`。
2. 执行 `GetErrorID()`，从报警表识别限位关节。
3. 执行 `DisableRobot()`，防止恢复期间自动运动。
4. 执行 `ClearError()`。
5. 执行 `RobotMode()`，打印当前 Dashboard 状态。
6. 在释放抱闸前等待人工按 Enter 确认。
7. 执行 `BrakeControl(N,1)`，只释放限位关节 N。
8. 等待人工移动该关节离开限位。
9. 人工按 Enter 后执行 `BrakeControl(N,0)` 重新抱闸。
10. 再次执行 `ClearError()`、`RobotMode()`、`GetErrorID()`。

脚本不会自动重新 enable。恢复后需要确认机械臂姿态、线缆和人员安全，再执行：

```bash
make enable
```

## 安全限制

- 释放抱闸前必须人工确认。
- 移动完成后必须人工确认才会重新抱闸。
- 如果脚本在释放抱闸后异常退出，会在 `finally` 中尽量执行 `BrakeControl(N,0)`。
- 自动识别失败时不会猜测关节，必须通过 `JOINT:=1..6` 指定。

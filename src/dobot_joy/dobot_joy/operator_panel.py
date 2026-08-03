"""Qt operator display for joystick collection; it never commands the robot."""

import json
import math
import os
import sys
import time
from collections import deque

os.environ.setdefault("MPLCONFIGDIR", "/tmp/dobot_operator_panel_mpl")

from dobot_interfaces.msg import (
    CartesianServoCommand,
    DobotState,
    GripperStatus,
    TeleopAction,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)


GUIDE = """Hand Controller\n\nY: enable / disable (locked while recording)\nRB: enter / exit drag\nX hold 1.5 s: save current start pose\nStart hold 1.5 s: open gripper, return to start pose\nStart tap: start recording\nBack tap: stop / review / queue LeRobot export\nBack hold 2 s: reject raw episode\nLB + sticks: ServoP motion\nB: emergency stop\n\nThe panel is read-only. Robot motion remains on the hand controller."""


class OperatorPanelNode(Node):
    def __init__(self):
        super().__init__("dobot_operator_panel")
        self.joy = {"axes": [], "buttons": []}
        self.robot = {}
        self.gripper = {}
        self.servo = {}
        self.collection = {}
        self.joint_history = deque(maxlen=300)
        self.action_history = deque(maxlen=300)
        self._history_start = time.monotonic()
        self._last_status_request = 0.0
        self._status_client = self.create_client(Trigger, "/data_collection/status")
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(DobotState, "/dobot_state", self._on_robot, 10)
        self.create_subscription(GripperStatus, "/gripper_state", self._on_gripper, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(TeleopAction, "/joy/teleop_action", self._on_action, 10)
        self.create_subscription(
            CartesianServoCommand, "/cartesian_servo/applied", self._on_servo, 10
        )

    def _on_joy(self, message):
        self.joy = {"axes": list(message.axes), "buttons": list(message.buttons)}

    def _on_robot(self, message):
        self.robot = {
            "enabled": bool(message.enable_status == 1),
            "mode": str(message.robot_mode_text),
            "error": int(message.error_status),
        }

    def _on_gripper(self, message):
        self.gripper = {
            "opening_mm": float(message.opening_mm),
            "moving": bool(message.moving),
            "object": bool(message.object_detected),
        }

    def _on_servo(self, message):
        self.servo = {
            "active": bool(message.active),
            "deadman": bool(message.deadman),
            "status": str(message.status),
            "velocity": list(message.normalized_velocity),
        }

    def _on_joints(self, message):
        if len(message.position) < 6:
            return
        stamp = time.monotonic() - self._history_start
        values = [math.degrees(float(value)) for value in message.position[:6]]
        self.joint_history.append((stamp, values))

    def _on_action(self, message):
        stamp = time.monotonic() - self._history_start
        values = [float(value) for value in message.cartesian_jog[:6]]
        values.append(float(message.gripper_target_normalized))
        self.action_history.append((stamp, values))

    def refresh_collection_status(self):
        if time.monotonic() - self._last_status_request < 0.75:
            return
        if not self._status_client.service_is_ready():
            return
        self._last_status_request = time.monotonic()
        future = self._status_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_status)

    def _on_status(self, future):
        try:
            response = future.result()
            if response.success:
                self.collection = json.loads(response.message)
        except Exception as exc:  # pragma: no cover - GUI callback safety
            self.collection = {"phase": "unavailable", "error": str(exc)}


class OperatorPanel(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Dobot ServoP Operator Panel")
        self.resize(1440, 980)
        root = QWidget()
        layout = QGridLayout(root)
        self.robot_label = QLabel()
        self.collection_label = QLabel()
        self.servo_label = QLabel()
        self.gripper_label = QLabel()
        self.joy_label = QLabel()
        self.export_status_label = QLabel()
        self.button_labels = {}
        for label in (
            self.robot_label,
            self.collection_label,
            self.servo_label,
            self.gripper_label,
            self.joy_label,
        ):
            label.setWordWrap(True)
            label.setStyleSheet("font-size: 15px; padding: 8px;")
        layout.addWidget(self._box("Robot", self.robot_label), 0, 0)
        layout.addWidget(self._box("Collection", self.collection_label), 0, 1)
        layout.addWidget(self._box("ServoP", self.servo_label), 1, 0)
        layout.addWidget(self._box("Gripper", self.gripper_label), 1, 1)
        layout.addWidget(self._box("Controller Input", self.joy_label), 2, 0, 1, 2)
        layout.addWidget(self._box("LeRobot Conversion", self.export_status_label), 2, 3)
        guide = QPlainTextEdit(GUIDE)
        guide.setReadOnly(True)
        guide.setStyleSheet("font-size: 14px;")
        layout.addWidget(self._box("Operation Guide", guide), 0, 3, 2, 1)

        self.joint_canvas, self.joint_axes, self.joint_lines = self._make_plot(
            "Joint position (deg)", [f"J{i}" for i in range(1, 7)]
        )
        self.action_canvas, self.action_axes, self.action_lines = self._make_plot(
            "Action values", ["X", "Y", "Z", "Rx", "Ry", "Rz", "Gripper"]
        )
        layout.addWidget(self._box("Joint Axes / Time", self.joint_canvas), 3, 0, 1, 2)
        layout.addWidget(self._box("Action / Time", self.action_canvas), 3, 2, 1, 2)

        button_panel = QWidget()
        button_layout = QGridLayout(button_panel)
        for column, (name, index) in enumerate(
            (("LB", 4), ("RB", 5), ("X", 2), ("Y", 3), ("Start", 7), ("Back", 6), ("B", 1))
        ):
            label = QLabel(f"{name} [{index}]\nup")
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumWidth(78)
            self.button_labels[name] = (label, index)
            button_layout.addWidget(label, 0, column)
        layout.addWidget(self._box("Controller State", button_panel), 4, 0, 1, 4)
        self.setCentralWidget(root)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

    @staticmethod
    def _box(title, widget):
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        layout.addWidget(widget)
        return box

    @staticmethod
    def _make_plot(title, names):
        figure = Figure(figsize=(6, 3), tight_layout=True)
        axes = figure.add_subplot(111)
        axes.set_title(title)
        axes.set_xlabel("time (s)")
        axes.grid(True, alpha=0.25)
        lines = []
        for name in names:
            line, = axes.plot([], [], label=name, linewidth=1.2)
            lines.append(line)
        axes.legend(loc="upper left", ncol=4, fontsize=8)
        return FigureCanvasQTAgg(figure), axes, lines

    @staticmethod
    def _values(values):
        return " ".join(f"{value:+.2f}" for value in values)

    def _update_plot(self, history, axes, lines):
        if not history:
            return
        samples = list(history)
        times = [sample[0] for sample in samples]
        values = list(zip(*(sample[1] for sample in samples)))
        for line, series in zip(lines, values):
            line.set_data(times, series)
        axes.set_xlim(max(0.0, times[-1] - 30.0), max(30.0, times[-1]))
        axes.relim()
        axes.autoscale_view(scalex=False, scaley=True)
        axes.figure.canvas.draw_idle()

    def _update_button_state(self, buttons):
        for name, (label, index) in self.button_labels.items():
            pressed = index < len(buttons) and bool(buttons[index])
            state = "PRESSED" if pressed else "up"
            color = "#b91c1c" if pressed else "#334155"
            label.setText(f"{name} [{index}]\n{state}")
            label.setStyleSheet(
                f"color: white; background: {color}; padding: 6px; border-radius: 4px;"
            )

    def _refresh(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.refresh_collection_status()
        robot = self.node.robot
        collection = self.node.collection
        servo = self.node.servo
        gripper = self.node.gripper
        joy = self.node.joy
        self.robot_label.setText(
            f"Enabled: {robot.get('enabled', False)}\n"
            f"Mode: {robot.get('mode', 'waiting')}\nError: {robot.get('error', '-') }"
        )
        self.collection_label.setText(
            f"Phase: {collection.get('phase', 'waiting')}\n"
            f"Samples: {collection.get('sample_count', 0)}\n"
            f"Return: {collection.get('return_phase', 'idle')}\n"
            f"Export: {collection.get('export_phase', 'idle')} "
            f"(queue {collection.get('export_queue_depth', 0)})"
        )
        self.servo_label.setText(
            f"Active: {servo.get('active', False)}  Deadman: {servo.get('deadman', False)}\n"
            f"Status: {servo.get('status', 'waiting')}\n"
            f"Velocity: {self._values(servo.get('velocity', []))}"
        )
        self.gripper_label.setText(
            f"Opening: {gripper.get('opening_mm', 0.0):.1f} mm\n"
            f"Moving: {gripper.get('moving', False)}  Object: {gripper.get('object', False)}"
        )
        self.joy_label.setText(
            f"Axes: {self._values(joy.get('axes', []))}\n"
            f"Buttons: {' '.join(str(value) for value in joy.get('buttons', []))}"
        )
        export = collection.get("last_lerobot_export") or {}
        phase = collection.get("export_phase", "idle")
        queue_depth = collection.get("export_queue_depth", 0)
        if phase == "exporting" or queue_depth:
            state, color = "RUNNING", "#f59e0b"
        elif export.get("exported") is True:
            state, color = "SUCCESS", "#16a34a"
        elif export.get("error") or phase == "failed":
            state, color = "FAILED", "#dc2626"
        else:
            state, color = "WAITING", "#64748b"
        item_colors = {
            "queued": "#f59e0b",
            "exporting": "#f59e0b",
            "success": "#16a34a",
            "failed": "#dc2626",
        }
        items = "<br>".join(
            f"<span style='color:{item_colors.get(item.get('status'), '#64748b')}'>"
            f"{item.get('status', 'unknown').upper()}</span> "
            f"{item.get('episode', '')}"
            for item in collection.get("export_items", [])[-8:]
        ) or "-"
        self.export_status_label.setText(
            f"<b style='color:{color}'>{state}</b><br>"
            f"phase: {phase}<br>queue: {queue_depth}<br>"
            f"current: {collection.get('export_current_dir', '') or '-'}<br>"
            f"{items}"
        )
        self._update_button_state(joy.get("buttons", []))
        self._update_plot(self.node.joint_history, self.joint_axes, self.joint_lines)
        self._update_plot(self.node.action_history, self.action_axes, self.action_lines)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorPanelNode()
    app = QApplication(sys.argv)
    panel = OperatorPanel(node)
    panel.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

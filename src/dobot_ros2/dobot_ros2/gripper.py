import time
from dataclasses import dataclass
from typing import Callable, Optional


REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_INIT_STATE = 0x0200
REG_GRIP_STATE = 0x0201
REG_POSITION_FEEDBACK = 0x0202

INIT_COMMAND_HOME = 0x01
INIT_COMMAND_CALIBRATE = 0xA5

INIT_STATE_NOT_INITIALIZED = 0
INIT_STATE_SUCCESS = 1
INIT_STATE_RUNNING = 2

GRIP_STATE_MOVING = 0
GRIP_STATE_REACHED = 1
GRIP_STATE_OBJECT_DETECTED = 2
GRIP_STATE_OBJECT_DROPPED = 3


@dataclass
class GripperConfig:
    enabled: bool = False
    transport: str = "dobot_modbus"
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    slave_id: int = 1
    modbus_ip: str = "127.0.0.1"
    modbus_port: int = 60000
    modbus_index: int = -1
    timeout_sec: float = 0.2
    stroke_mm: float = 95.0
    max_force_n: float = 160.0
    default_force_percent: int = 50
    min_force_percent: int = 20
    max_force_percent: int = 100
    auto_connect: bool = True


@dataclass
class GripperResult:
    success: bool
    message: str
    init_state: int = -1
    grip_state: int = -1
    position_permille: int = -1
    opening_mm: float = -1.0
    force_percent: int = -1
    connected: bool = False

    @property
    def initialized(self) -> bool:
        return self.init_state == INIT_STATE_SUCCESS

    @property
    def moving(self) -> bool:
        return self.grip_state == GRIP_STATE_MOVING

    @property
    def object_detected(self) -> bool:
        return self.grip_state == GRIP_STATE_OBJECT_DETECTED

    @property
    def object_dropped(self) -> bool:
        return self.grip_state == GRIP_STATE_OBJECT_DROPPED


class DobotModbusAgGripper:
    """DH Robotics AG gripper through Dobot controller Modbus RTU bridge."""

    def __init__(self, config: GripperConfig, command: Callable[[str, str], object]):
        self.config = config
        self._command = command
        self._index = int(config.modbus_index)

    def connect(self) -> GripperResult:
        if not self.config.enabled:
            return GripperResult(False, "gripper disabled")
        if self.is_connected():
            return GripperResult(True, "gripper modbus already connected", connected=True)
        result = self._command(
            (
                f"ModbusCreate({self.config.modbus_ip},{self.config.modbus_port:d},"
                f"{self.config.slave_id:d},1)"
            ),
            "gripper_modbus_create",
        )
        if not result.success:
            return GripperResult(
                False,
                f"gripper modbus create failed: {result.raw_reply or result.message}",
            )
        values = _reply_ints(result.raw_reply)
        if not values:
            return GripperResult(False, f"gripper modbus index missing: {result.raw_reply}")
        self._index = values[0]
        return GripperResult(True, f"gripper modbus connected index={self._index}", connected=True)

    def disconnect(self) -> None:
        if self._index < 0:
            return
        self._command(f"ModbusClose({self._index:d})", "gripper_modbus_close")
        self._index = -1

    def is_connected(self) -> bool:
        return self._index >= 0

    def initialize(self, calibrate: bool = False) -> GripperResult:
        ensure = self._ensure_connected()
        if not ensure.success:
            return ensure
        command = INIT_COMMAND_CALIBRATE if calibrate else INIT_COMMAND_HOME
        result = self._write_register(REG_INIT, command)
        if not result.success:
            return result
        return self.state(prefix="gripper initialize accepted")

    def move(
        self,
        opening_mm: float,
        position_permille: int,
        force_percent: int,
        force_n: float,
        wait: bool,
        timeout_sec: float,
    ) -> GripperResult:
        ensure = self._ensure_connected()
        if not ensure.success:
            return ensure

        target = self._target_permille(opening_mm, position_permille)
        force = self._target_force_percent(force_percent, force_n)
        force_result = self._write_register(REG_FORCE, force)
        if not force_result.success:
            return force_result
        position_result = self._write_register(REG_POSITION, target)
        if not position_result.success:
            return position_result
        if wait:
            return self._wait_for_motion(timeout_sec)
        return self.state(prefix="gripper move accepted")

    def state(self, prefix: str = "gripper state") -> GripperResult:
        ensure = self._ensure_connected()
        if not ensure.success:
            return ensure
        init_state = self._read_register(REG_INIT_STATE)
        if not init_state.success:
            return init_state
        grip_state = self._read_register(REG_GRIP_STATE)
        if not grip_state.success:
            return grip_state
        position = self._read_register(REG_POSITION_FEEDBACK)
        if not position.success:
            return position
        force = self._read_register(REG_FORCE)
        force_value = force.force_percent if force.success else -1
        return GripperResult(
            True,
            prefix,
            init_state=init_state.init_state,
            grip_state=grip_state.grip_state,
            position_permille=position.position_permille,
            opening_mm=self._permille_to_mm(position.position_permille),
            force_percent=force_value,
            connected=self.is_connected(),
        )

    def _wait_for_motion(self, timeout_sec: float) -> GripperResult:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while True:
            result = self.state(prefix="gripper move complete")
            if not result.success:
                return result
            if result.grip_state in (
                GRIP_STATE_REACHED,
                GRIP_STATE_OBJECT_DETECTED,
                GRIP_STATE_OBJECT_DROPPED,
            ):
                return result
            if time.monotonic() >= deadline:
                result.success = False
                result.message = "gripper move timeout"
                return result
            time.sleep(0.05)

    def _ensure_connected(self) -> GripperResult:
        if not self.config.enabled:
            return GripperResult(False, "gripper disabled")
        if self.is_connected():
            return GripperResult(True, "gripper modbus connected", connected=True)
        if not self.config.auto_connect:
            return GripperResult(False, "gripper modbus not connected")
        return self.connect()

    def _read_register(self, register: int) -> GripperResult:
        result = self._command(
            f"GetHoldRegs({self._index:d},{register:d},1,U16)",
            "gripper_modbus_read",
        )
        if not result.success:
            return GripperResult(False, f"read 0x{register:04X} failed: {result.raw_reply}")
        values = _reply_ints(result.raw_reply)
        if not values:
            return GripperResult(False, f"read 0x{register:04X} returned no value: {result.raw_reply}")
        value = values[0]
        response = GripperResult(True, f"read register 0x{register:04X}", connected=True)
        if register == REG_INIT_STATE:
            response.init_state = value
        elif register == REG_GRIP_STATE:
            response.grip_state = value
        elif register == REG_POSITION_FEEDBACK:
            response.position_permille = value
            response.opening_mm = self._permille_to_mm(value)
        elif register == REG_FORCE:
            response.force_percent = value
        return response

    def _write_register(self, register: int, value: int) -> GripperResult:
        result = self._command(
            f"SetHoldRegs({self._index:d},{register:d},1,{{{int(value):d}}},U16)",
            "gripper_modbus_write",
        )
        if not result.success:
            return GripperResult(False, f"write 0x{register:04X} failed: {result.raw_reply}")
        return GripperResult(True, f"write register 0x{register:04X} accepted", connected=True)

    def _target_permille(self, opening_mm: float, position_permille: int) -> int:
        if opening_mm >= 0.0:
            value = round(opening_mm / self.config.stroke_mm * 1000.0)
        else:
            value = position_permille
        return self._clamp_int(value, 0, 1000)

    def _target_force_percent(self, force_percent: int, force_n: float) -> int:
        if force_n >= 0.0 and self.config.max_force_n > 0.0:
            value = round(force_n / self.config.max_force_n * 100.0)
        elif force_percent > 0:
            value = force_percent
        else:
            value = self.config.default_force_percent
        return self._clamp_int(
            value,
            self.config.min_force_percent,
            self.config.max_force_percent,
        )

    def _permille_to_mm(self, position_permille: int) -> float:
        if position_permille < 0:
            return -1.0
        return float(position_permille) * self.config.stroke_mm / 1000.0

    @staticmethod
    def _clamp_int(value: int, lower: int, upper: int) -> int:
        return max(lower, min(upper, int(value)))


class DhAgGripper:
    """DH Robotics AG gripper controller using Modbus-RTU over RS485."""

    def __init__(self, config: GripperConfig):
        self.config = config
        self._serial = None

    def connect(self) -> GripperResult:
        if not self.config.enabled:
            return GripperResult(False, "gripper disabled")
        if self.is_connected():
            return GripperResult(True, "gripper already connected", connected=True)
        try:
            import serial
        except ImportError as exc:
            return GripperResult(False, f"pyserial is not installed: {exc}")
        try:
            self._serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baudrate,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=1,
                timeout=self.config.timeout_sec,
            )
        except Exception as exc:
            self._serial = None
            return GripperResult(False, f"failed to open gripper serial port: {exc}")
        return GripperResult(True, "gripper connected", connected=True)

    def disconnect(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def is_connected(self) -> bool:
        return bool(self._serial is not None and self._serial.is_open)

    def initialize(self, calibrate: bool = False) -> GripperResult:
        ensure = self._ensure_connected()
        if not ensure.success:
            return ensure
        command = INIT_COMMAND_CALIBRATE if calibrate else INIT_COMMAND_HOME
        result = self._write_register(REG_INIT, command)
        if not result.success:
            return result
        return self.state(prefix="gripper initialize accepted")

    def move(
        self,
        opening_mm: float,
        position_permille: int,
        force_percent: int,
        force_n: float,
        wait: bool,
        timeout_sec: float,
    ) -> GripperResult:
        ensure = self._ensure_connected()
        if not ensure.success:
            return ensure

        target = self._target_permille(opening_mm, position_permille)
        force = self._target_force_percent(force_percent, force_n)
        force_result = self._write_register(REG_FORCE, force)
        if not force_result.success:
            return force_result
        position_result = self._write_register(REG_POSITION, target)
        if not position_result.success:
            return position_result
        if wait:
            return self._wait_for_motion(timeout_sec)
        return self.state(prefix="gripper move accepted")

    def state(self, prefix: str = "gripper state") -> GripperResult:
        ensure = self._ensure_connected()
        if not ensure.success:
            return ensure
        init_state = self._read_register(REG_INIT_STATE)
        if not init_state.success:
            return init_state
        grip_state = self._read_register(REG_GRIP_STATE)
        if not grip_state.success:
            return grip_state
        position = self._read_register(REG_POSITION_FEEDBACK)
        if not position.success:
            return position
        force = self._read_register(REG_FORCE)
        force_value = force.force_percent if force.success else -1
        return GripperResult(
            True,
            prefix,
            init_state=init_state.init_state,
            grip_state=grip_state.grip_state,
            position_permille=position.position_permille,
            opening_mm=self._permille_to_mm(position.position_permille),
            force_percent=force_value,
            connected=self.is_connected(),
        )

    def _wait_for_motion(self, timeout_sec: float) -> GripperResult:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while True:
            result = self.state(prefix="gripper move complete")
            if not result.success:
                return result
            if result.grip_state in (
                GRIP_STATE_REACHED,
                GRIP_STATE_OBJECT_DETECTED,
                GRIP_STATE_OBJECT_DROPPED,
            ):
                return result
            if time.monotonic() >= deadline:
                result.success = False
                result.message = "gripper move timeout"
                return result
            time.sleep(0.05)

    def _ensure_connected(self) -> GripperResult:
        if not self.config.enabled:
            return GripperResult(False, "gripper disabled")
        if self.is_connected():
            return GripperResult(True, "gripper connected", connected=True)
        if not self.config.auto_connect:
            return GripperResult(False, "gripper not connected")
        return self.connect()

    def _read_register(self, register: int) -> GripperResult:
        request = bytes([
            self.config.slave_id,
            0x03,
            (register >> 8) & 0xFF,
            register & 0xFF,
            0x00,
            0x01,
        ])
        response = self._transact(request, 7)
        if response is None:
            return GripperResult(False, f"no gripper response for register 0x{register:04X}")
        if len(response) != 7 or response[1] != 0x03 or response[2] != 0x02:
            return GripperResult(False, f"invalid gripper response: {response.hex(' ')}")
        value = (response[3] << 8) | response[4]
        result = GripperResult(True, f"read register 0x{register:04X}", connected=True)
        if register == REG_INIT_STATE:
            result.init_state = value
        elif register == REG_GRIP_STATE:
            result.grip_state = value
        elif register == REG_POSITION_FEEDBACK:
            result.position_permille = value
            result.opening_mm = self._permille_to_mm(value)
        elif register == REG_FORCE:
            result.force_percent = value
        return result

    def _write_register(self, register: int, value: int) -> GripperResult:
        request = bytes([
            self.config.slave_id,
            0x06,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])
        response = self._transact(request, 8)
        if response is None:
            return GripperResult(False, f"no gripper response for register 0x{register:04X}")
        if response != request + _crc(request):
            return GripperResult(False, f"invalid gripper write echo: {response.hex(' ')}")
        return GripperResult(True, f"write register 0x{register:04X} accepted", connected=True)

    def _transact(self, payload: bytes, response_size: int) -> Optional[bytes]:
        frame = payload + _crc(payload)
        self._serial.reset_input_buffer()
        self._serial.write(frame)
        response = self._serial.read(response_size)
        if len(response) != response_size or not _valid_crc(response):
            return None
        return response

    def _target_permille(self, opening_mm: float, position_permille: int) -> int:
        if opening_mm >= 0.0:
            value = round(opening_mm / self.config.stroke_mm * 1000.0)
        else:
            value = position_permille
        return self._clamp_int(value, 0, 1000)

    def _target_force_percent(self, force_percent: int, force_n: float) -> int:
        if force_n >= 0.0 and self.config.max_force_n > 0.0:
            value = round(force_n / self.config.max_force_n * 100.0)
        elif force_percent > 0:
            value = force_percent
        else:
            value = self.config.default_force_percent
        return self._clamp_int(
            value,
            self.config.min_force_percent,
            self.config.max_force_percent,
        )

    def _permille_to_mm(self, position_permille: int) -> float:
        if position_permille < 0:
            return -1.0
        return float(position_permille) * self.config.stroke_mm / 1000.0

    @staticmethod
    def _clamp_int(value: int, lower: int, upper: int) -> int:
        return max(lower, min(upper, int(value)))


def _crc(payload: bytes) -> bytes:
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _valid_crc(frame: bytes) -> bool:
    return len(frame) >= 3 and _crc(frame[:-2]) == frame[-2:]


def _reply_ints(reply: str) -> list:
    start = reply.find("{")
    end = reply.find("}", start + 1)
    if start < 0 or end < 0:
        return []
    values = []
    for item in reply[start + 1 : end].split(","):
        item = item.strip()
        if not item:
            continue
        try:
            values.append(int(float(item)))
        except ValueError:
            pass
    return values

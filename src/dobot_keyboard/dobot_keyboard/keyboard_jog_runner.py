import argparse
import os
import signal
import subprocess
import sys
import time


def _stop_jog():
    command = [
        "ros2",
        "service",
        "call",
        "/move_jog",
        "dobot_interfaces/srv/JogCommand",
        "{stop: true}",
    ]
    try:
        subprocess.run(command, timeout=3.0, check=False)
    except Exception as exc:
        print(f"keyboard jog stop failed: {exc}", file=sys.stderr)


def _terminate_process_group(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 3.0
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--params-file", default="")
    parser.add_argument("--input-topic", default="/keyboard/input")
    parser.add_argument("--device", default="/dev/input/event0")
    parser.add_argument("--jog-coord-type", default="0")
    parser.add_argument("--user", default="0")
    parser.add_argument("--tool", default="0")
    args = parser.parse_args(argv)

    launch_command = [
        "ros2",
        "launch",
        "dobot_keyboard",
        "keyboard_jog.launch.py",
        f"params_file:={args.params_file}",
        f"input_topic:={args.input_topic}",
        f"device:={args.device}",
        f"jog_coord_type:={args.jog_coord_type}",
        f"user:={args.user}",
        f"tool:={args.tool}",
    ]
    process = subprocess.Popen(launch_command, start_new_session=True)
    interrupt_count = 0

    def handle_sigint(signum, frame):
        del signum, frame
        nonlocal interrupt_count
        interrupt_count += 1
        _stop_jog()
        if interrupt_count == 1:
            print("keyboard jog stopped; press Ctrl+C again to exit", flush=True)
            return
        _terminate_process_group(process)

    signal.signal(signal.SIGINT, handle_sigint)
    try:
        while process.poll() is None:
            time.sleep(0.1)
        return int(process.returncode or 0)
    finally:
        _stop_jog()
        _terminate_process_group(process)


if __name__ == "__main__":
    raise SystemExit(main())

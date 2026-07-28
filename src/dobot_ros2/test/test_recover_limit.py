import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from dobot_ros2.recover_limit import (  # noqa: E402
    detect_limit_joint,
    parse_error_ids,
)


def test_parse_error_ids_from_dashboard_reply():
    reply = "0,{[[75],[],[],[],[],[],[]]},GetErrorID();"

    assert parse_error_ids(reply) == [75]


def test_detect_limit_joint_from_alarm_description():
    assert detect_limit_joint([75], PACKAGE_ROOT / "files") == 6
    assert detect_limit_joint([69], PACKAGE_ROOT / "files") == 3
    assert detect_limit_joint([], PACKAGE_ROOT / "files") is None


def test_makefile_exposes_interactive_limit_recovery():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "recover-limit:" in source
    assert "ROBOT_IP ?= 192.168.5.1" in source
    assert "DASHBOARD_PORT ?= 29999" in source
    assert "ros2 run dobot_ros2 dobot_recover_limit" in source
    assert "--joint \"$(JOINT)\"" in source

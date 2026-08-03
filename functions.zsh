# Dobot Nova2 operator functions.
# Usage:
#   cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
#   source functions.zsh
#
# These functions are the public operator interface and keep related commands
# under the dobot-* prefix. Their dispatcher is self-contained.

typeset -g DOBOT_REPO_DIR="${DOBOT_REPO_DIR:-${${(%):-%N}:A:h}}"

_dobot_command() {
  emulate -L zsh
  local target="$1"
  shift
  if [[ -z "${target}" ]]; then
    print -u2 "Usage: _dobot_command <command> [VAR=value ...]"
    return 2
  fi
  "${DOBOT_REPO_DIR}/scripts/dobot-command.sh" "${target}" "$@"
}

dobot-help() {
  print "Dobot Nova2 daily commands:"
  print "  Setup:"
  print "    dobot-build"
  print "  ServoP collection:"
  print "    dobot-servo-task"
  print "    dobot-servo-collect [SYSTEM_VIEW=true]"
  print "    dobot-data-set-start"
  print "    dobot-data-prepare"
  print "    dobot-data-status"
  print "    dobot-data-validate"
  print "    dobot-lerobot-setup"
  print "  OpenPI policy:"
  print "    dobot-policy-train"
  print "    dobot-policy-dry-run"
  print "    dobot-policy-real"
  print "    dobot-policy-status"
  print "    dobot-policy-stop"
  print "  Robot safety and gripper:"
  print "    dobot-state"
  print "    dobot-errors"
  print "    dobot-clear"
  print "    dobot-enable"
  print "    dobot-disable"
  print "    dobot-estop"
  print "    dobot-gripper-state"
  print "    dobot-gripper-open"
  print "    dobot-gripper-close"
  print "  Diagnostics:"
  print "    dobot-camera-check"
  print "    dobot-camera-view"
  print "    dobot-logs-latest"
  print "    dobot-tf"
  print "    dobot-frames"
}

dobot-build() { _dobot_command build "$@" }
dobot-driver() { _dobot_command driver "$@" }
dobot-bringup() { _dobot_command bringup "$@" }
dobot-system() { _dobot_command system "$@" }
dobot-rviz() { _dobot_command rviz "$@" }
dobot-control-ui() { _dobot_command control-ui "$@" }
dobot-control-ui-only() { _dobot_command control-ui-only "$@" }
dobot-services() { _dobot_command services "$@" }
dobot-topics() { _dobot_command topics "$@" }
dobot-tf() { _dobot_command tf "$@" }
dobot-frames() { _dobot_command frames "$@" }

dobot-servo-task() { _dobot_command servo-data-task "$@" }
dobot-servo-collect() { _dobot_command servo-collect "$@" }
dobot-lerobot-setup() { _dobot_command lerobot-setup "$@" }
dobot-data-status() { _dobot_command data-status "$@" }
dobot-data-prepare() { _dobot_command data-prepare "$@" }
dobot-data-set-start() { _dobot_command data-set-start "$@" }
dobot-data-accept() { _dobot_command data-accept "$@" }
dobot-data-reject() { _dobot_command data-reject "$@" }
dobot-data-validate() { _dobot_command data-lerobot-validate "$@" }
dobot-data-validate-episode() { _dobot_command data-validate "$@" }
dobot-data-clear-start() { _dobot_command data-clear-start "$@" }
dobot-data-stop() { _dobot_command data-stop "$@" }
dobot-data-lerobot-export() { _dobot_command data-lerobot-export "$@" }

dobot-policy-train() { _dobot_command policy-train "$@" }
dobot-policy-real() { _dobot_command policy-real "$@" }
dobot-policy-dry-run() { _dobot_command policy-dry-run "$@" }
dobot-policy-motion-test() { _dobot_command policy-motion-test "$@" }
dobot-policy-status() { _dobot_command policy-status "$@" }
dobot-policy-stop() { _dobot_command policy-stop "$@" }

dobot-state() { _dobot_command state "$@" }
dobot-joints() { _dobot_command joints "$@" }
dobot-tcp() { _dobot_command tcp "$@" }
dobot-errors() { _dobot_command errors "$@" }
dobot-clear() { _dobot_command clear "$@" }
dobot-enable() { _dobot_command enable "$@" }
dobot-disable() { _dobot_command disable "$@" }
dobot-estop() { _dobot_command estop "$@" }
dobot-drag-start() { _dobot_command drag-start "$@" }
dobot-drag-stop() { _dobot_command drag-stop "$@" }
dobot-recover-limit() { _dobot_command recover-limit "$@" }

dobot-gripper-init() { _dobot_command gripper-init "$@" }
dobot-gripper-state() { _dobot_command gripper-state "$@" }
dobot-gripper-open() { _dobot_command gripper-open "$@" }
dobot-gripper-close() { _dobot_command gripper-close "$@" }
dobot-gripper-move() { _dobot_command gripper-move "$@" }

dobot-camera() { _dobot_command camera "$@" }
dobot-camera-wrist() { _dobot_command camera-wrist "$@" }
dobot-camera-global() { _dobot_command camera-global "$@" }
dobot-camera-view() { _dobot_command camera-view "$@" }
dobot-camera-topics() { _dobot_command camera-topics "$@" }
dobot-camera-info() { _dobot_command camera-info "$@" }
dobot-camera-check() { _dobot_command camera-check "$@" }
dobot-logs-latest() { _dobot_command logs-latest "$@" }

dobot-handeye-capture() { _dobot_command handeye-capture "$@" }
dobot-handeye-check() { _dobot_command handeye-check "$@" }
dobot-handeye-solve() { _dobot_command handeye-solve "$@" }
dobot-handeye-validate() { _dobot_command handeye-validate "$@" }
dobot-handeye-diagnose() { _dobot_command handeye-diagnose "$@" }
dobot-handeye-tf() { _dobot_command handeye-tf "$@" }
dobot-handeye-board-tf() { _dobot_command handeye-board-tf "$@" }
dobot-servo-data-lerobot-validate() { _dobot_command servo-data-lerobot-validate "$@" }

dobot-keyboard() { _dobot_command keyboard "$@" }
dobot-keyboard-jog() { _dobot_command keyboard-jog "$@" }
dobot-keyboard-input() { _dobot_command keyboard-input "$@" }
dobot-keyboard-jog-input() { _dobot_command keyboard-jog-input "$@" }
dobot-keyboard-teleop() { _dobot_command keyboard-teleop "$@" }
dobot-joy() { _dobot_command joy "$@" }
dobot-joy-teleop() { _dobot_command joy-teleop "$@" }

dobot-move-jog() { _dobot_command move-jog "$@" }
dobot-jog-stop() { _dobot_command jog-stop "$@" }
dobot-teach-start() { _dobot_command teach-start "$@" }
dobot-teach-stop() { _dobot_command teach-stop "$@" }
dobot-teach-replay() { _dobot_command teach-replay "$@" }
dobot-teach-replay-servoj() { _dobot_command teach-replay-servoj "$@" }
dobot-teach-list() { _dobot_command teach-list "$@" }
dobot-teach-delete() { _dobot_command teach-delete "$@" }
dobot-teach-status() { _dobot_command teach-status "$@" }
dobot-movej() { _dobot_command movej "$@" }
dobot-movejp() { _dobot_command movejp "$@" }
dobot-movel() { _dobot_command movel "$@" }
dobot-movep() { _dobot_command movep "$@" }

# Print the command index every time this file is sourced, as requested for
# operator terminals and handoff sessions.
dobot-help

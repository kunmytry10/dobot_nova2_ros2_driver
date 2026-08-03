# Dobot Nova2 operator functions.
# Usage:
#   cd /home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver
#   source functions.zsh
#
# Make remains the compatibility backend; these functions are the public
# operator interface and keep related commands under the dobot-* prefix.

typeset -g DOBOT_REPO_DIR="${DOBOT_REPO_DIR:-${${(%):-%N}:A:h}}"

_dobot_make() {
  emulate -L zsh
  local target="$1"
  shift
  if [[ -z "${target}" ]]; then
    print -u2 "Usage: _dobot_make <make-target> [VAR=value ...]"
    return 2
  fi
  (
    cd "${DOBOT_REPO_DIR}" || return 1
    command make "${target}" "$@"
  )
}

dobot-help() {
  print "Dobot Nova2 functions (source functions.zsh):"
  print "  Environment:"
  print "    dobot-build                 Build the ROS packages"
  print "    dobot-system                Start robot, cameras, ServoP and Qt panel"
  print "    dobot-driver / dobot-bringup Start driver or driver + TF"
  print "  ServoP data:"
  print "    dobot-servo-task            Write task from config/pi05_pipeline.env"
  print "    dobot-servo-collect         Start ServoP collection"
  print "    dobot-lerobot-setup          Install the pinned LeRobot v3 tools"
  print "    dobot-data-status            Show collection and export queue"
  print "    dobot-data-prepare           Return to the saved start pose"
  print "    dobot-data-validate           Validate the LeRobot v3 dataset"
  print "  OpenPI policy:"
  print "    dobot-policy-train           Start Docker training from the pipeline config"
  print "    dobot-policy-dry-run         Run inference without robot motion"
  print "    dobot-policy-real            Start the warm real-robot policy session"
  print "    dobot-policy-status / stop   Inspect or stop the policy node"
  print "    dobot-policy-motion-demo     Run the saved motion demo"
  print "  Robot and gripper:"
  print "    dobot-state / joints / tcp   Read robot feedback"
  print "    dobot-errors / clear         Read or clear controller alarms"
  print "    dobot-enable / disable       Enable or disable the robot"
  print "    dobot-estop                  Software emergency stop"
  print "    dobot-gripper-init/state    Initialize or inspect the gripper"
  print "    dobot-gripper-open/close    Open or close the gripper"
  print "  Cameras and logs:"
  print "    dobot-camera                 Start both cameras"
  print "    dobot-camera-view/check      View images or check frame rates"
  print "    dobot-logs-latest            Locate the newest system log"
  print "  Hand-eye:"
  print "    dobot-handeye-capture/solve/validate/tf"
  print ""
  print "Most functions accept Make-style overrides, for example:"
  print "  dobot-policy-real POLICY_MAX_EPISODE_SEC=120"
}

dobot-build() { _dobot_make build "$@" }
dobot-driver() { _dobot_make driver "$@" }
dobot-bringup() { _dobot_make bringup "$@" }
dobot-system() { _dobot_make system "$@" }
dobot-rviz() { _dobot_make rviz "$@" }
dobot-control-ui() { _dobot_make control-ui "$@" }
dobot-control-ui-only() { _dobot_make control-ui-only "$@" }
dobot-services() { _dobot_make services "$@" }
dobot-topics() { _dobot_make topics "$@" }
dobot-tf() { _dobot_make tf "$@" }
dobot-frames() { _dobot_make frames "$@" }

dobot-servo-task() { _dobot_make servo-data-task "$@" }
dobot-servo-collect() { _dobot_make servo-collect "$@" }
dobot-lerobot-setup() { _dobot_make lerobot-setup "$@" }
dobot-data-status() { _dobot_make data-status "$@" }
dobot-data-prepare() { _dobot_make data-prepare "$@" }
dobot-data-set-start() { _dobot_make data-set-start "$@" }
dobot-data-accept() { _dobot_make data-accept "$@" }
dobot-data-reject() { _dobot_make data-reject "$@" }
dobot-data-validate() { _dobot_make data-lerobot-validate "$@" }
dobot-data-validate-episode() { _dobot_make data-validate "$@" }
dobot-data-task() { _dobot_make data-task "$@" }
dobot-data-clear-start() { _dobot_make data-clear-start "$@" }
dobot-data-start-pose-status() { _dobot_make data-start-pose-status "$@" }
dobot-data-start() { _dobot_make data-start "$@" }
dobot-data-stop() { _dobot_make data-stop "$@" }
dobot-data-lerobot-export() { _dobot_make data-lerobot-export "$@" }

dobot-policy-train() { _dobot_make policy-train "$@" }
dobot-policy-real() { _dobot_make policy-real "$@" }
dobot-policy-dry-run() { _dobot_make policy-dry-run "$@" }
dobot-policy-motion-test() { _dobot_make policy-motion-test "$@" }
dobot-policy-motion-full() { _dobot_make policy-motion-full "$@" }
dobot-policy-motion-demo() { _dobot_make policy-motion-demo "$@" }
dobot-policy-demo-stop() { _dobot_make policy-demo-stop "$@" }
dobot-policy-status() { _dobot_make policy-status "$@" }
dobot-policy-stop() { _dobot_make policy-stop "$@" }

dobot-state() { _dobot_make state "$@" }
dobot-joints() { _dobot_make joints "$@" }
dobot-tcp() { _dobot_make tcp "$@" }
dobot-errors() { _dobot_make errors "$@" }
dobot-clear() { _dobot_make clear "$@" }
dobot-enable() { _dobot_make enable "$@" }
dobot-disable() { _dobot_make disable "$@" }
dobot-estop() { _dobot_make estop "$@" }
dobot-drag-start() { _dobot_make drag-start "$@" }
dobot-drag-stop() { _dobot_make drag-stop "$@" }
dobot-recover-limit() { _dobot_make recover-limit "$@" }

dobot-gripper-init() { _dobot_make gripper-init "$@" }
dobot-gripper-state() { _dobot_make gripper-state "$@" }
dobot-gripper-open() { _dobot_make gripper-open "$@" }
dobot-gripper-close() { _dobot_make gripper-close "$@" }
dobot-gripper-move() { _dobot_make gripper-move "$@" }

dobot-camera() { _dobot_make camera "$@" }
dobot-camera-wrist() { _dobot_make camera-wrist "$@" }
dobot-camera-global() { _dobot_make camera-global "$@" }
dobot-camera-view() { _dobot_make camera-view "$@" }
dobot-camera-topics() { _dobot_make camera-topics "$@" }
dobot-camera-info() { _dobot_make camera-info "$@" }
dobot-camera-check() { _dobot_make camera-check "$@" }
dobot-logs-latest() { _dobot_make logs-latest "$@" }

dobot-handeye-capture() { _dobot_make handeye-capture "$@" }
dobot-handeye-check() { _dobot_make handeye-check "$@" }
dobot-handeye-solve() { _dobot_make handeye-solve "$@" }
dobot-handeye-validate() { _dobot_make handeye-validate "$@" }
dobot-handeye-diagnose() { _dobot_make handeye-diagnose "$@" }
dobot-handeye-tf() { _dobot_make handeye-tf "$@" }
dobot-handeye-board-tf() { _dobot_make handeye-board-tf "$@" }
dobot-servo-data-lerobot-validate() { _dobot_make servo-data-lerobot-validate "$@" }

dobot-keyboard() { _dobot_make keyboard "$@" }
dobot-keyboard-jog() { _dobot_make keyboard-jog "$@" }
dobot-keyboard-input() { _dobot_make keyboard-input "$@" }
dobot-keyboard-jog-input() { _dobot_make keyboard-jog-input "$@" }
dobot-keyboard-teleop() { _dobot_make keyboard-teleop "$@" }
dobot-joy() { _dobot_make joy "$@" }
dobot-joy-teleop() { _dobot_make joy-teleop "$@" }

dobot-move-jog() { _dobot_make move-jog "$@" }
dobot-jog-stop() { _dobot_make jog-stop "$@" }
dobot-teach-start() { _dobot_make teach-start "$@" }
dobot-teach-stop() { _dobot_make teach-stop "$@" }
dobot-teach-replay() { _dobot_make teach-replay "$@" }
dobot-teach-replay-servoj() { _dobot_make teach-replay-servoj "$@" }
dobot-teach-list() { _dobot_make teach-list "$@" }
dobot-teach-delete() { _dobot_make teach-delete "$@" }
dobot-teach-status() { _dobot_make teach-status "$@" }
dobot-movej() { _dobot_make movej "$@" }
dobot-movejp() { _dobot_make movejp "$@" }
dobot-movel() { _dobot_make movel "$@" }
dobot-movep() { _dobot_make movep "$@" }

# Print the command index every time this file is sourced, as requested for
# operator terminals and handoff sessions.
dobot-help

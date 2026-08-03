#!/usr/bin/env bash
# Direct command backend for the public dobot-* shell functions.
set -euo pipefail

ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command_name="${1:-}"
[[ -n "$command_name" ]] || { echo "Usage: dobot-command.sh <command> [KEY=value ...]" >&2; exit 2; }
shift
command_overrides=("$@")

config_file="${COLLECTION_CONFIG:-$ws/config/pi05_pipeline.env}"
[[ -r "$config_file" ]] && source "$config_file"
for assignment in "$@"; do
  assignment="${assignment/:=/=}"
  [[ "$assignment" =~ ^([A-Za-z_][A-Za-z0-9_]*)=(.*)$ ]] || { echo "Invalid override: $assignment" >&2; exit 2; }
  printf -v "${BASH_REMATCH[1]}" '%s' "${BASH_REMATCH[2]}"
done

: "${ORBBEC_WS:=$HOME/orbbec_305}" "${PARAMS:=$ws/src/dobot_ros2/config/dobot_ros2.yaml}"
: "${ROBOT_IP:=192.168.5.1}" "${DASHBOARD_PORT:=29999}" "${JOINT:=0}" "${U:=0}" "${T:=0}"
: "${SPEED:=2}" "${ACC:=2}" "${WAIT:=true}" "${TIMEOUT:=20.0}" "${J:=[]}" "${P:=[]}" "${AXIS:=}" "${TRAJ:=}" "${OVERWRITE:=false}" "${REPLAY_MODE:=}"
: "${CONSOLE_HOST:=0.0.0.0}" "${CONSOLE_PORT:=8080}" "${GRIPPER_OPENING_MM:=-1.0}" "${GRIPPER_POS:=1000}" "${GRIPPER_FORCE:=50}" "${GRIPPER_FORCE_N:=-1.0}"
: "${CAMERA_LAUNCH:=gemini_330_series.launch.py}" "${CAMERA_NAME:=camera}" "${CAMERA_SERIAL:=}" "${CAMERA_USB_PORT:=}"
: "${REALSENSE_NAME:=global_camera}" "${REALSENSE_SERIAL:=}" "${REALSENSE_USB_PORT:=}" "${REALSENSE_COLOR_PROFILE:=640,480,30}" "${REALSENSE_ENABLE_DEPTH:=false}"
: "${DATASET:=}" "${EPISODE:=}" "${UV:=$HOME/.local/bin/uv}" "${LEROBOT_COMMIT:=95211b98f1cd6b638bda84a8d28f9e41323229dd}"
: "${DATA_ROOT:=$ws/data}" "${HANDEYE_DATASET_ROOT:=$DATA_ROOT/handeye/datasets}" "${HANDEYE_DATASET_NAME:=}" "${HANDEYE_SAMPLES_DIR:=$DATA_ROOT/handeye/samples}" "${HANDEYE_RESULT_FILE:=}" "${HANDEYE_DIAGNOSE_FILE:=}"
: "${HANDEYE_STATIC_TF_FILE:=$DATA_ROOT/handeye/handeye_result.yaml}" "${HANDEYE_STATIC_TF_CHILD_FRAME:=camera_link}" "${HANDEYE_PARENT_FRAME:=Link6}" "${HANDEYE_CHILD_FRAME:=camera_color_optical_frame}" "${HANDEYE_METHOD:=TSAI}"
: "${KEYBOARD_TOPIC:=/keyboard/input}" "${KEYBOARD_DEV:=/dev/input/event0}" "${KEYBOARD_STEP_MM:=5.0}" "${KEYBOARD_ROT_STEP_DEG:=2.0}" "${KEYBOARD_MOTION_SERVICE:=movep}" "${KEYBOARD_GRIPPER_INIT:=true}" "${KEYBOARD_JOG_COORD_TYPE:=0}"
: "${JOY_TOPIC:=/joy}" "${JOY_DIAGNOSTICS_TOPIC:=/joy/teleop_diagnostics}" "${JOY_DEV:=/dev/input/js0}" "${JOY_DEADMAN_BUTTON:=4}" "${JOY_ESTOP_BUTTON:=1}" "${JOY_TOGGLE_ENABLE_BUTTON:=3}" "${JOY_TOGGLE_DRAG_BUTTON:=5}" "${JOY_DEADZONE:=0.25}" "${JOY_CONTROL_MODE:=move_jog}" "${JOY_RESPONSE_EXPONENT:=1.2}" "${JOY_COORD_TYPE:=0}" "${JOY_AUTOREPEAT_RATE:=50.0}" "${JOY_GRIPPER_INIT:=true}"
: "${JOY_GRIPPER_STEP_MM:=2.0}" "${JOY_GRIPPER_STOP_LEAD_MM:=3.0}" "${JOY_GRIPPER_FORCE:=50}" "${JOY_ENABLE_RUMBLE:=true}" "${JOY_JOINT_LIMIT_MARGIN_DEG:=0.0}" "${JOY_X_AXIS_INDEX:=1}" "${JOY_Y_AXIS_INDEX:=0}" "${JOY_RX_AXIS_INDEX:=6}" "${JOY_RY_AXIS_INDEX:=7}" "${JOY_X_AXIS_SIGN:=-1.0}" "${JOY_Y_AXIS_SIGN:=-1.0}" "${JOY_Z_AXIS_SIGN:=1.0}" "${JOY_RZ_AXIS_SIGN:=-1.0}" "${JOY_RX_AXIS_SIGN:=-1.0}" "${JOY_RY_AXIS_SIGN:=-1.0}"
: "${JOY_START_DATA_COLLECTION:=true}" "${JOY_DATASET_ROOT:=$DATA_ROOT/collections/move_jog}" "${JOY_WRIST_IMAGE_TOPIC:=/camera/color/image_raw}" "${JOY_WRIST_CAMERA_INFO_TOPIC:=/camera/color/camera_info}" "${JOY_GLOBAL_IMAGE_TOPIC:=/global_camera/color/image_raw}" "${JOY_GLOBAL_CAMERA_INFO_TOPIC:=/global_camera/color/camera_info}" "${JOY_DATA_SAMPLE_RATE_HZ:=10.0}"
: "${COLLECTION_REQUIRE_START_POSE:=false}" "${COLLECTION_START_POSE_FILE:=$JOY_DATASET_ROOT/servo_p_start_pose.json}" "${COLLECTION_START_JOINT_TOLERANCE_DEG:=1.0}" "${COLLECTION_START_GRIPPER_TOLERANCE_MM:=3.0}" "${COLLECTION_PREPARE_SPEED:=10}" "${COLLECTION_PREPARE_ACCELERATION:=10}" "${COLLECTION_PREPARE_TIMEOUT_SEC:=30.0}"
: "${SERVO_DATASET_ROOT:=$DATA_ROOT/collections/servo_p_v2}" "${SERVO_LEROBOT_DATASET_ROOT:=$SERVO_DATASET_ROOT/lerobot_pi05_servo_p_v2}" "${SERVO_LEROBOT_REPO_ID:=local/dobot_nova2_tape_pi05_servo_p_v2}" "${SERVO_START_POSE_FILE:=$SERVO_DATASET_ROOT/servo_p_start_pose.json}" "${SERVO_TASK:=}" "${JOY_TASK:=}" "${JOY_TASK_FILE:=$JOY_DATASET_ROOT/current_task.txt}"
: "${JOY_LEROBOT_ENABLED:=true}" "${JOY_LEROBOT_PYTHON:=$ws/.venv-lerobot/bin/python}" "${JOY_LEROBOT_DATASET_ROOT:=$JOY_DATASET_ROOT/lerobot_tape_pi05}" "${JOY_LEROBOT_REPO_ID:=local/dobot_nova2_tape_pi05}" "${JOY_LEROBOT_EXPORT_TIMEOUT_SEC:=900.0}" "${JOY_MAX_IMAGE_SKEW_SEC:=0.05}" "${JOY_DATA_REJECT_HOLD_SEC:=2.0}" "${JOY_COLLECTION_PREPARE_HOLD_SEC:=1.5}" "${JOY_LIMIT_RECOVERY_HOLD_SEC:=3.0}" "${JOY_LIMIT_RECOVERY_TIMEOUT_SEC:=10.0}"
: "${ROBOT_MODE:=bringup}" "${SYSTEM_VIEW:=false}" "${SYSTEM_START_CAMERA:=true}" "${SYSTEM_START_JOY:=true}" "${SYSTEM_START_DATA_COLLECTION:=true}" "${SYSTEM_INIT_GRIPPER:=true}" "${SYSTEM_LOG_ROOT:=$ws/logs}" "${SYSTEM_LOCK_FILE:=/tmp/dobot_nova2_system_$(id -u).lock}"

# Generated ROS setup scripts can reference optional variables before defining
# them, so do not apply nounset while loading those scripts.
source_ros_environment() {
  set +u
  source /opt/ros/humble/setup.bash
  [[ ! -f "$ORBBEC_WS/install/setup.bash" ]] || source "$ORBBEC_WS/install/setup.bash"
  set -u
}
ros() (
  source_ros_environment
  cd "$ws"
  set +u
  source install/setup.bash
  set -u
  "$@"
)
optional_arg() { [[ -n "$2" ]] && printf '%s:=%s\n' "$1" "$2"; }
gripper_init() { [[ "$JOY_GRIPPER_INIT" != true ]] || { ros timeout 10s ros2 service call /gripper_init std_srvs/srv/Trigger '{}' || true; }; }
data_service() { ros ros2 service call "/data_collection/$1" std_srvs/srv/Trigger '{}'; }

case "$command_name" in
  build) source_ros_environment; cd "$ws"; colcon build --symlink-install --packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_joy dobot_ros2 dobot_policy ;;
  driver) ros ros2 run dobot_ros2 dobot_motion_server --ros-args --params-file "$PARAMS" ;;
  bringup|rviz) ros ros2 launch dobot_ros2 dobot_bringup.launch.py params_file:="$PARAMS" "rviz:=$([[ $command_name == rviz ]] && echo true || echo false)" handeye_result_file:="$HANDEYE_STATIC_TF_FILE" handeye_output_child_frame:="$HANDEYE_STATIC_TF_CHILD_FRAME" ;;
  control-ui) ros ros2 launch dobot_ros2 dobot_control_console.launch.py params_file:="$PARAMS" console_host:="$CONSOLE_HOST" console_port:="$CONSOLE_PORT" start_driver:=true start_state_publisher:=true handeye_result_file:="$HANDEYE_STATIC_TF_FILE" handeye_output_child_frame:="$HANDEYE_STATIC_TF_CHILD_FRAME" ;;
  control-ui-only) ros ros2 launch dobot_ros2 dobot_control_console.launch.py params_file:="$PARAMS" console_host:="$CONSOLE_HOST" console_port:="$CONSOLE_PORT" start_driver:=false start_state_publisher:=false ;;
  services) ros bash -c 'ros2 service list | grep -E "get_robot_state|get_joint_state|get_tcp_pose|get_gripper_state|clear_error|enable_robot|disable_robot|emergency_stop|drag_start|drag_stop|get_error_id|gripper|teach|move"' ;;
  topics) ros bash -c 'ros2 topic list | grep -E "^/joint_states$|^/tcp_pose$|^/dobot_state$|^/gripper_state$|^/keyboard/input$|^/joy$|^/tf$|^/tf_static$"' ;;
  tf) ros bash -c 'ros2 topic list | grep -E "^/tf$|^/tf_static$"' ;;
  frames) ros ros2 run tf2_tools view_frames ;;
  state) ros ros2 service call /get_robot_state dobot_interfaces/srv/GetRobotState '{}' ;;
  errors) ros ros2 service call /get_error_id std_srvs/srv/Trigger '{}' ;;
  clear) ros ros2 service call /clear_error std_srvs/srv/Trigger '{}' ;;
  enable) ros ros2 service call /enable_robot std_srvs/srv/Trigger '{}' ;;
  disable) ros ros2 service call /disable_robot std_srvs/srv/Trigger '{}' ;;
  estop) ros ros2 service call /emergency_stop std_srvs/srv/Trigger '{}' ;;
  drag-start) ros ros2 service call /drag_start std_srvs/srv/Trigger '{}' ;;
  drag-stop) ros ros2 service call /drag_stop std_srvs/srv/Trigger '{}' ;;
  recover-limit) ros ros2 run dobot_ros2 dobot_recover_limit --robot-ip "$ROBOT_IP" --port "$DASHBOARD_PORT" --joint "$JOINT" ;;
  joints) ros ros2 service call /get_joint_state dobot_interfaces/srv/GetJointState '{}' ;;
  tcp) ros ros2 service call /get_tcp_pose dobot_interfaces/srv/GetTcpPose '{}' ;;
  gripper-init) ros ros2 service call /gripper_init std_srvs/srv/Trigger '{}' ;;
  gripper-state) ros ros2 service call /get_gripper_state dobot_interfaces/srv/GripperState '{}' ;;
  gripper-open) ros ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand "{opening_mm: 95.0, force_percent: $GRIPPER_FORCE, force_n: $GRIPPER_FORCE_N, wait: $WAIT, timeout_sec: $TIMEOUT}" ;;
  gripper-close) ros ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand "{opening_mm: 0.0, force_percent: $GRIPPER_FORCE, force_n: $GRIPPER_FORCE_N, wait: $WAIT, timeout_sec: $TIMEOUT}" ;;
  gripper-move) ros ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand "{opening_mm: $GRIPPER_OPENING_MM, position_permille: $GRIPPER_POS, force_percent: $GRIPPER_FORCE, force_n: $GRIPPER_FORCE_N, wait: $WAIT, timeout_sec: $TIMEOUT}" ;;
  camera) ros ros2 launch dobot_camera dual_camera.launch.py orbbec_launch_file:="$CAMERA_LAUNCH" wrist_camera_name:="$CAMERA_NAME" global_camera_name:="$REALSENSE_NAME" global_color_profile:="$REALSENSE_COLOR_PROFILE" global_enable_depth:="$REALSENSE_ENABLE_DEPTH" $(optional_arg wrist_serial_number "$CAMERA_SERIAL") $(optional_arg wrist_usb_port "$CAMERA_USB_PORT") $(optional_arg global_serial_no "$REALSENSE_SERIAL") $(optional_arg global_usb_port_id "$REALSENSE_USB_PORT") ;;
  camera-wrist) ros ros2 launch dobot_camera gemini305.launch.py orbbec_launch_file:="$CAMERA_LAUNCH" camera_name:="$CAMERA_NAME" $(optional_arg serial_number "$CAMERA_SERIAL") $(optional_arg usb_port "$CAMERA_USB_PORT") ;;
  camera-global) ros ros2 launch dobot_camera realsense_global.launch.py camera_name:="$REALSENSE_NAME" rgb_camera.color_profile:="$REALSENSE_COLOR_PROFILE" enable_depth:="$REALSENSE_ENABLE_DEPTH" $(optional_arg serial_no "$REALSENSE_SERIAL") $(optional_arg usb_port_id "$REALSENSE_USB_PORT") ;;
  camera-view) ros ros2 launch dobot_camera camera_view.launch.py ;;
  camera-topics) ros bash -c 'ros2 topic list | grep -E "^/camera/|^/global_camera/"' ;;
  camera-info) ros bash -c 'ros2 topic echo /camera/color/camera_info --once && ros2 topic echo /global_camera/color/camera_info --once' ;;
  camera-check) ros bash -c 'timeout 6s ros2 topic hz /camera/color/image_raw || true; timeout 6s ros2 topic hz /global_camera/color/image_raw || true' ;;
  handeye-check) ros ros2 run dobot_handeye dobot_handeye_check --ros-args --params-file "$PARAMS" ;;
  handeye-capture) ros ros2 run dobot_handeye dobot_handeye_capture --dataset-root "$HANDEYE_DATASET_ROOT" --dataset-name "$HANDEYE_DATASET_NAME" --ros-args --params-file "$PARAMS" ;;
  handeye-solve) ros ros2 run dobot_handeye dobot_handeye_solve --dataset "$DATASET" --samples-dir "$HANDEYE_SAMPLES_DIR" --result-file "$HANDEYE_RESULT_FILE" --parent-frame "$HANDEYE_PARENT_FRAME" --child-frame "$HANDEYE_CHILD_FRAME" --method "$HANDEYE_METHOD" ;;
  handeye-validate) ros ros2 run dobot_handeye dobot_handeye_validate --dataset "$DATASET" --result-file "$HANDEYE_RESULT_FILE" ;;
  handeye-diagnose) ros ros2 run dobot_handeye dobot_handeye_diagnose --dataset "$DATASET" --diagnose-file "$HANDEYE_DIAGNOSE_FILE" ;;
  handeye-tf) ros ros2 run dobot_handeye dobot_handeye_tf --dataset "$DATASET" --result-file "$HANDEYE_RESULT_FILE" --output-child-frame "$HANDEYE_STATIC_TF_CHILD_FRAME" ;;
  handeye-board-tf) ros ros2 run dobot_handeye dobot_handeye_board_tf --ros-args --params-file "$PARAMS" ;;
  keyboard) gripper_init; ros ros2 launch dobot_keyboard keyboard_teleop.launch.py params_file:="$PARAMS" input_topic:="$KEYBOARD_TOPIC" translation_step_mm:="$KEYBOARD_STEP_MM" rotation_step_deg:="$KEYBOARD_ROT_STEP_DEG" motion_service:="$KEYBOARD_MOTION_SERVICE" speed:="$SPEED" acceleration:="$ACC" wait:="$WAIT" timeout_sec:="$TIMEOUT" ;;
  keyboard-jog) gripper_init; ros ros2 run dobot_keyboard dobot_keyboard_jog_runner --params-file "$PARAMS" --input-topic "$KEYBOARD_TOPIC" --device "$KEYBOARD_DEV" --jog-coord-type "$KEYBOARD_JOG_COORD_TYPE" --user "$U" --tool "$T" ;;
  keyboard-input) ros ros2 run dobot_keyboard dobot_keyboard_input --ros-args -p input_topic:="$KEYBOARD_TOPIC" ;;
  keyboard-jog-input) ros ros2 run dobot_keyboard dobot_keyboard_jog_input --ros-args -p input_topic:="$KEYBOARD_TOPIC" -p device:="$KEYBOARD_DEV" ;;
  keyboard-teleop) ros ros2 run dobot_keyboard dobot_keyboard_teleop --ros-args --params-file "$PARAMS" -p keyboard.input_topic:="$KEYBOARD_TOPIC" -p keyboard.translation_step_mm:="$KEYBOARD_STEP_MM" -p keyboard.rotation_step_deg:="$KEYBOARD_ROT_STEP_DEG" -p keyboard.motion_service:="$KEYBOARD_MOTION_SERVICE" -p keyboard.speed:="$SPEED" -p keyboard.acceleration:="$ACC" -p keyboard.wait:="$WAIT" -p keyboard.timeout_sec:="$TIMEOUT" ;;
  system)
    exec 9>"$SYSTEM_LOCK_FILE"; flock -n 9 || { echo "ERROR: another system is already running (lock: $SYSTEM_LOCK_FILE)" >&2; exit 2; }
    run_dir="$SYSTEM_LOG_ROOT/run_$(date +%Y%m%d_%H%M%S)_$BASHPID"; mkdir -p "$run_dir/ros"; echo "system log: $run_dir"
    ros bash -c 'export ROS_LOG_DIR="$1" RCUTILS_COLORIZED_OUTPUT=1; shift; exec ros2 launch dobot_ros2 dobot_system.launch.py "$@"' bash "$run_dir/ros" robot_mode:="$ROBOT_MODE" start_camera:="$SYSTEM_START_CAMERA" start_view:="$SYSTEM_VIEW" start_joy:="$SYSTEM_START_JOY" start_data_collection:="$SYSTEM_START_DATA_COLLECTION" start_operator_panel:="$SYSTEM_START_OPERATOR_PANEL" init_gripper:="$SYSTEM_INIT_GRIPPER" run_log_dir:="$run_dir" params_file:="$PARAMS" handeye_result_file:="$HANDEYE_STATIC_TF_FILE" orbbec_launch_file:="$CAMERA_LAUNCH" wrist_camera_name:="$CAMERA_NAME" $(optional_arg wrist_serial_number "$CAMERA_SERIAL") $(optional_arg wrist_usb_port "$CAMERA_USB_PORT") global_camera_name:="$REALSENSE_NAME" $(optional_arg global_serial_no "$REALSENSE_SERIAL") $(optional_arg global_usb_port_id "$REALSENSE_USB_PORT") global_color_profile:="$REALSENSE_COLOR_PROFILE" global_enable_depth:="$REALSENSE_ENABLE_DEPTH" joy_topic:="$JOY_TOPIC" joy_dev:="$JOY_DEV" joy_autorepeat_rate:="$JOY_AUTOREPEAT_RATE" dataset_root:="$JOY_DATASET_ROOT" wrist_image_topic:="$JOY_WRIST_IMAGE_TOPIC" wrist_camera_info_topic:="$JOY_WRIST_CAMERA_INFO_TOPIC" global_image_topic:="$JOY_GLOBAL_IMAGE_TOPIC" global_camera_info_topic:="$JOY_GLOBAL_CAMERA_INFO_TOPIC" data_sample_rate_hz:="$JOY_DATA_SAMPLE_RATE_HZ" collection_require_start_pose:="$COLLECTION_REQUIRE_START_POSE" collection_start_pose_file:="$COLLECTION_START_POSE_FILE" collection_start_joint_tolerance_deg:="$COLLECTION_START_JOINT_TOLERANCE_DEG" collection_start_gripper_tolerance_mm:="$COLLECTION_START_GRIPPER_TOLERANCE_MM" collection_prepare_speed:="$COLLECTION_PREPARE_SPEED" collection_prepare_acceleration:="$COLLECTION_PREPARE_ACCELERATION" collection_prepare_timeout_sec:="$COLLECTION_PREPARE_TIMEOUT_SEC" max_image_skew_sec:="$JOY_MAX_IMAGE_SKEW_SEC" task_file:="$JOY_TASK_FILE" lerobot_enabled:="$JOY_LEROBOT_ENABLED" lerobot_python:="$JOY_LEROBOT_PYTHON" lerobot_dataset_root:="$JOY_LEROBOT_DATASET_ROOT" lerobot_repo_id:="$JOY_LEROBOT_REPO_ID" lerobot_export_timeout_sec:="$JOY_LEROBOT_EXPORT_TIMEOUT_SEC" diagnostics_topic:="$JOY_DIAGNOSTICS_TOPIC" deadman_button_index:="$JOY_DEADMAN_BUTTON" estop_button_index:="$JOY_ESTOP_BUTTON" toggle_enable_button_index:="$JOY_TOGGLE_ENABLE_BUTTON" toggle_drag_button_index:="$JOY_TOGGLE_DRAG_BUTTON" collection_prepare_hold_sec:="$JOY_COLLECTION_PREPARE_HOLD_SEC" deadzone:="$JOY_DEADZONE" control_mode:="$JOY_CONTROL_MODE" response_exponent:="$JOY_RESPONSE_EXPONENT" coord_type:="$JOY_COORD_TYPE" x_axis_index:="$JOY_X_AXIS_INDEX" y_axis_index:="$JOY_Y_AXIS_INDEX" rx_axis_index:="$JOY_RX_AXIS_INDEX" ry_axis_index:="$JOY_RY_AXIS_INDEX" x_axis_sign:="$JOY_X_AXIS_SIGN" y_axis_sign:="$JOY_Y_AXIS_SIGN" z_axis_sign:="$JOY_Z_AXIS_SIGN" rz_axis_sign:="$JOY_RZ_AXIS_SIGN" rx_axis_sign:="$JOY_RX_AXIS_SIGN" ry_axis_sign:="$JOY_RY_AXIS_SIGN" gripper_step_mm:="$JOY_GRIPPER_STEP_MM" gripper_stop_lead_mm:="$JOY_GRIPPER_STOP_LEAD_MM" gripper_force_percent:="$JOY_GRIPPER_FORCE" enable_rumble:="$JOY_ENABLE_RUMBLE" joint_limit_margin_deg:="$JOY_JOINT_LIMIT_MARGIN_DEG" limit_recovery_hold_sec:="$JOY_LIMIT_RECOVERY_HOLD_SEC" limit_recovery_release_timeout_sec:="$JOY_LIMIT_RECOVERY_TIMEOUT_SEC" data_reject_hold_sec:="$JOY_DATA_REJECT_HOLD_SEC" 2>&1 | tee "$run_dir/launch.log" ;;
  servo-collect) "$0" system "${command_overrides[@]}" SYSTEM_START_OPERATOR_PANEL=true JOY_CONTROL_MODE=servo_p COLLECTION_REQUIRE_START_POSE=true JOY_DATASET_ROOT="$SERVO_DATASET_ROOT" JOY_TASK_FILE="$SERVO_DATASET_ROOT/current_task.txt" JOY_LEROBOT_DATASET_ROOT="$SERVO_LEROBOT_DATASET_ROOT" JOY_LEROBOT_REPO_ID="$SERVO_LEROBOT_REPO_ID" COLLECTION_START_POSE_FILE="$SERVO_START_POSE_FILE" ;;
  servo-data-task) "$0" data-task "${command_overrides[@]}" JOY_DATASET_ROOT="$SERVO_DATASET_ROOT" JOY_TASK_FILE="$SERVO_DATASET_ROOT/current_task.txt" TASK="$SERVO_TASK" ;;
  servo-data-lerobot-validate) "$0" data-lerobot-validate "${command_overrides[@]}" JOY_DATASET_ROOT="$SERVO_DATASET_ROOT" JOY_LEROBOT_DATASET_ROOT="$SERVO_LEROBOT_DATASET_ROOT" JOY_LEROBOT_REPO_ID="$SERVO_LEROBOT_REPO_ID" ;;
  logs-latest) find "$SYSTEM_LOG_ROOT" -mindepth 1 -maxdepth 1 -type d -name 'run_*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2- | { read -r result || true; [[ -n "${result:-}" ]] && echo "$result" || echo 'no system logs'; } ;;
  joy)
    gripper_init
    joy_args=(joy_topic:="$JOY_TOPIC" diagnostics_topic:="$JOY_DIAGNOSTICS_TOPIC" dev:="$JOY_DEV" autorepeat_rate:="$JOY_AUTOREPEAT_RATE" deadman_button_index:="$JOY_DEADMAN_BUTTON" estop_button_index:="$JOY_ESTOP_BUTTON" toggle_enable_button_index:="$JOY_TOGGLE_ENABLE_BUTTON" toggle_drag_button_index:="$JOY_TOGGLE_DRAG_BUTTON" deadzone:="$JOY_DEADZONE" control_mode:="$JOY_CONTROL_MODE" response_exponent:="$JOY_RESPONSE_EXPONENT" coord_type:="$JOY_COORD_TYPE" gripper_step_mm:="$JOY_GRIPPER_STEP_MM" gripper_stop_lead_mm:="$JOY_GRIPPER_STOP_LEAD_MM" gripper_force_percent:="$JOY_GRIPPER_FORCE" enable_rumble:="$JOY_ENABLE_RUMBLE" joint_limit_margin_deg:="$JOY_JOINT_LIMIT_MARGIN_DEG" x_axis_index:="$JOY_X_AXIS_INDEX" y_axis_index:="$JOY_Y_AXIS_INDEX" rx_axis_index:="$JOY_RX_AXIS_INDEX" ry_axis_index:="$JOY_RY_AXIS_INDEX" x_axis_sign:="$JOY_X_AXIS_SIGN" y_axis_sign:="$JOY_Y_AXIS_SIGN" z_axis_sign:="$JOY_Z_AXIS_SIGN" rz_axis_sign:="$JOY_RZ_AXIS_SIGN" rx_axis_sign:="$JOY_RX_AXIS_SIGN" ry_axis_sign:="$JOY_RY_AXIS_SIGN" start_data_collection:="$JOY_START_DATA_COLLECTION" dataset_root:="$JOY_DATASET_ROOT" wrist_image_topic:="$JOY_WRIST_IMAGE_TOPIC" wrist_camera_info_topic:="$JOY_WRIST_CAMERA_INFO_TOPIC" global_image_topic:="$JOY_GLOBAL_IMAGE_TOPIC" global_camera_info_topic:="$JOY_GLOBAL_CAMERA_INFO_TOPIC" data_sample_rate_hz:="$JOY_DATA_SAMPLE_RATE_HZ" collection_require_start_pose:="$COLLECTION_REQUIRE_START_POSE" collection_start_pose_file:="$COLLECTION_START_POSE_FILE" collection_start_joint_tolerance_deg:="$COLLECTION_START_JOINT_TOLERANCE_DEG" collection_start_gripper_tolerance_mm:="$COLLECTION_START_GRIPPER_TOLERANCE_MM" collection_prepare_speed:="$COLLECTION_PREPARE_SPEED" collection_prepare_acceleration:="$COLLECTION_PREPARE_ACCELERATION" collection_prepare_timeout_sec:="$COLLECTION_PREPARE_TIMEOUT_SEC" max_image_skew_sec:="$JOY_MAX_IMAGE_SKEW_SEC" task_file:="$JOY_TASK_FILE" lerobot_enabled:="$JOY_LEROBOT_ENABLED" lerobot_python:="$JOY_LEROBOT_PYTHON" lerobot_dataset_root:="$JOY_LEROBOT_DATASET_ROOT" lerobot_repo_id:="$JOY_LEROBOT_REPO_ID" lerobot_export_timeout_sec:="$JOY_LEROBOT_EXPORT_TIMEOUT_SEC" limit_recovery_hold_sec:="$JOY_LIMIT_RECOVERY_HOLD_SEC" limit_recovery_release_timeout_sec:="$JOY_LIMIT_RECOVERY_TIMEOUT_SEC" data_reject_hold_sec:="$JOY_DATA_REJECT_HOLD_SEC")
    [[ -z "$JOY_TASK" ]] || joy_args+=(task_instruction:="$JOY_TASK"); ros ros2 launch dobot_joy joy_teleop.launch.py "${joy_args[@]}" ;;
  joy-teleop) ros ros2 run dobot_joy dobot_joy_teleop --ros-args -p joy.topic:="$JOY_TOPIC" -p joy.deadman_button_index:="$JOY_DEADMAN_BUTTON" -p joy.estop_button_index:="$JOY_ESTOP_BUTTON" -p joy.toggle_enable_button_index:="$JOY_TOGGLE_ENABLE_BUTTON" -p joy.toggle_drag_button_index:="$JOY_TOGGLE_DRAG_BUTTON" -p joy.deadzone:="$JOY_DEADZONE" -p joy.control_mode:="$JOY_CONTROL_MODE" -p joy.response_exponent:="$JOY_RESPONSE_EXPONENT" -p joy.coord_type:="$JOY_COORD_TYPE" -p joy.gripper_step_mm:="$JOY_GRIPPER_STEP_MM" -p joy.gripper_stop_lead_mm:="$JOY_GRIPPER_STOP_LEAD_MM" -p joy.gripper_force_percent:="$JOY_GRIPPER_FORCE" -p joy.enable_rumble:="$JOY_ENABLE_RUMBLE" -p joy.joint_limit_margin_deg:="$JOY_JOINT_LIMIT_MARGIN_DEG" -p joy.data_reject_hold_sec:="$JOY_DATA_REJECT_HOLD_SEC" -p joy.x_axis_index:="$JOY_X_AXIS_INDEX" -p joy.y_axis_index:="$JOY_Y_AXIS_INDEX" -p joy.rx_axis_index:="$JOY_RX_AXIS_INDEX" -p joy.ry_axis_index:="$JOY_RY_AXIS_INDEX" -p joy.x_axis_sign:="$JOY_X_AXIS_SIGN" -p joy.y_axis_sign:="$JOY_Y_AXIS_SIGN" -p joy.z_axis_sign:="$JOY_Z_AXIS_SIGN" -p joy.rz_axis_sign:="$JOY_RZ_AXIS_SIGN" -p joy.rx_axis_sign:="$JOY_RX_AXIS_SIGN" -p joy.ry_axis_sign:="$JOY_RY_AXIS_SIGN" ;;
  lerobot-setup) "$UV" venv --python 3.12 "$ws/.venv-lerobot"; "$UV" pip install --python "$JOY_LEROBOT_PYTHON" "lerobot[dataset] @ git+https://github.com/huggingface/lerobot.git@$LEROBOT_COMMIT" ;;
  data-set-start) data_service set_start_pose ;;
  data-prepare) data_service prepare ;;
  data-clear-start) data_service clear_start_pose ;;
  data-start-pose-status|data-status) data_service status ;;
  data-start) data_service start ;;
  data-stop) data_service stop ;;
  data-accept) data_service accept ;;
  data-reject) data_service reject ;;
  data-task) mkdir -p "$(dirname "$JOY_TASK_FILE")"; [[ -z "${TASK:-}" ]] || printf '%s\n' "$TASK" > "$JOY_TASK_FILE"; [[ ! -f "$JOY_TASK_FILE" ]] && echo 'current data task is empty' || { printf 'current data task: '; cat "$JOY_TASK_FILE"; } ;;
  data-validate) ros ros2 run dobot_joy dobot_data_validate "$EPISODE" ;;
  data-lerobot-export) "$JOY_LEROBOT_PYTHON" "$ws/src/dobot_joy/dobot_joy/lerobot_export.py" export "$EPISODE" "$JOY_LEROBOT_DATASET_ROOT" "$JOY_LEROBOT_REPO_ID" "$JOY_DATA_SAMPLE_RATE_HZ" ;;
  data-lerobot-validate) "$JOY_LEROBOT_PYTHON" "$ws/src/dobot_joy/dobot_joy/lerobot_export.py" validate "$JOY_LEROBOT_DATASET_ROOT" "$JOY_LEROBOT_REPO_ID" ;;
  policy-real) "$ws/scripts/run_pi05_grasp.sh" ;;
  policy-train) PI05_TRAIN_CONFIG="${PI05_TRAIN_CONFIG:-$ws/config/pi05_pipeline.env}" "$ws/scripts/train_pi05_dobot.sh" ;;
  policy-dry-run) POLICY_ARMED=false POLICY_RUN_TIMEOUT_SEC=25 "$ws/scripts/run_pi05_grasp.sh" ;;
  policy-motion-test) POLICY_ARMED=true POLICY_MOTION_ONLY=true POLICY_RUN_TIMEOUT_SEC=40 "$ws/scripts/run_pi05_grasp.sh" ;;
  policy-motion-full) POLICY_ARMED=true POLICY_MOTION_ONLY=true POLICY_MOTION_TEST_DURATION_SEC=45 POLICY_MAX_EPISODE_SEC=75 POLICY_RUN_TIMEOUT_SEC=100 "$ws/scripts/run_pi05_grasp.sh" ;;
  policy-motion-demo) POLICY_ARMED=true POLICY_MOTION_ONLY=true POLICY_START_POSE_FILE="$SERVO_START_POSE_FILE" POLICY_MOTION_TEST_DURATION_SEC=45 POLICY_MAX_EPISODE_SEC=75 POLICY_RUN_TIMEOUT_SEC=100 POLICY_COUNTDOWN_SEC=3 POLICY_SKIP_BUILD=true POLICY_REUSE_SERVER=true POLICY_KEEP_SERVER=true "$ws/scripts/run_pi05_grasp.sh" ;;
  policy-demo-stop) POLICY_STOP_SERVER_ONLY=true "$ws/scripts/run_pi05_grasp.sh" ;;
  policy-status) ros ros2 service call /dobot_policy/status std_srvs/srv/Trigger '{}' ;;
  policy-stop) ros ros2 service call /dobot_policy/stop std_srvs/srv/Trigger '{}' ;;
  move-jog) ros ros2 service call /move_jog dobot_interfaces/srv/JogCommand "{axis_id: '$AXIS', stop: false, coord_type: $JOY_COORD_TYPE, user: $U, tool: $T}" ;;
  jog-stop) ros ros2 service call /move_jog dobot_interfaces/srv/JogCommand '{stop: true}' ;;
  teach-start) ros ros2 service call /teach_start dobot_interfaces/srv/TrajectoryCommand "{name: '$TRAJ', overwrite: $OVERWRITE}" ;;
  teach-stop) ros ros2 service call /teach_stop dobot_interfaces/srv/TrajectoryCommand "{name: '$TRAJ'}" ;;
  teach-replay) ros ros2 service call /teach_replay dobot_interfaces/srv/TrajectoryCommand "{name: '$TRAJ', speed: $SPEED, acceleration: $ACC, replay_mode: '$REPLAY_MODE', override_wait: true, wait: $WAIT, timeout_sec: $TIMEOUT}" ;;
  teach-replay-servoj) ros ros2 service call /teach_replay dobot_interfaces/srv/TrajectoryCommand "{name: '$TRAJ', speed: $SPEED, acceleration: $ACC, replay_mode: 'servoj', override_wait: true, wait: $WAIT, timeout_sec: $TIMEOUT}" ;;
  teach-list) ros ros2 service call /teach_list dobot_interfaces/srv/TrajectoryList '{}' ;;
  teach-delete) ros ros2 service call /teach_delete dobot_interfaces/srv/TrajectoryCommand "{name: '$TRAJ'}" ;;
  teach-status) ros ros2 service call /teach_status std_srvs/srv/Trigger '{}' ;;
  movej|movejp|movel|movep)
    target="$J"; [[ "$command_name" == movej ]] || target="$P"
    ros ros2 service call "/$command_name" dobot_interfaces/srv/MoveCommand "{target: $target, user: $U, tool: $T, speed: $SPEED, acceleration: $ACC, wait: $WAIT, timeout_sec: $TIMEOUT}" ;;
  *) echo "Unknown command: $command_name" >&2; exit 2 ;;
esac

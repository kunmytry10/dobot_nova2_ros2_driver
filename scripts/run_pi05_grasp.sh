#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${DOBOT_WORKSPACE_DIR:-/home/ps/DZK_repos/dobot/dobot_nova2_ros2_driver}"
OPENPI_DIR="${OPENPI_REPO_DIR:-/home/ps/DZK_repos/openpi}"
# Keep this Compose project aligned with OpenPI's functions.zsh helpers so the
# deployment reuses the already-running development container for this user.
export OPENPI_COMPOSE_PROJECT_NAME="${OPENPI_COMPOSE_PROJECT_NAME:-openpi-dev-$(id -u)}"
POLICY_PYTHON="${POLICY_PYTHON:-${WORKSPACE_DIR}/.venv-policy/bin/python}"
UV_BIN="${UV_BIN:-/home/ps/.local/bin/uv}"
POLICY_CONFIG="${OPENPI_POLICY_CONFIG:-pi05_dobot_pen_box_servo_p_action_only}"
CHECKPOINT_DIR="${OPENPI_CHECKPOINT_DIR:-${OPENPI_DIR}/../openpi-docker-data/checkpoints/pi05_dobot_pen_box_servo_p_action_only/dobot_pen_box_servo_p_action_only_v1_long/135000}"
CHECKPOINT_CONTAINER_DIR="${OPENPI_CHECKPOINT_CONTAINER_DIR:-/workspace/checkpoints/pi05_dobot_pen_box_servo_p_action_only/dobot_pen_box_servo_p_action_only_v1_long/135000}"
POLICY_PORT="${OPENPI_POLICY_PORT:-8000}"
COUNTDOWN_SEC="${POLICY_COUNTDOWN_SEC:-2}"
RUN_TIMEOUT_SEC="${POLICY_RUN_TIMEOUT_SEC:-120}"
POLICY_ARMED="${POLICY_ARMED:-true}"
POLICY_MOTION_ONLY="${POLICY_MOTION_ONLY:-false}"
POLICY_MOTION_TEST_DURATION_SEC="${POLICY_MOTION_TEST_DURATION_SEC:-3.0}"
POLICY_MAX_EPISODE_SEC="${POLICY_MAX_EPISODE_SEC:-90.0}"
POLICY_START_POSE_FILE="${POLICY_START_POSE_FILE:-${WORKSPACE_DIR}/data_collection_servo_p_v2/servo_p_start_pose.json}"
POLICY_SKIP_BUILD="${POLICY_SKIP_BUILD:-true}"
POLICY_REUSE_SERVER="${POLICY_REUSE_SERVER:-true}"
POLICY_KEEP_SERVER="${POLICY_KEEP_SERVER:-true}"
POLICY_STOP_SERVER_ONLY="${POLICY_STOP_SERVER_ONLY:-false}"
POLICY_INTERACTIVE="${POLICY_INTERACTIVE:-true}"
POLICY_LOG_DIR="${WORKSPACE_DIR}/logs/policy"
SERVER_LOG="${OPENPI_DIR}/../openpi-docker-data/wandb/dobot_pen_box_servo_p_action_only_v1_long_deploy.log"
SERVER_META_FILE="/tmp/openpi_pi05_policy_server_$(id -u).meta"
LOCK_FILE="/tmp/dobot_nova2_system_$(id -u).lock"
ORBBEC_WS="${ORBBEC_WS:-${HOME}/orbbec_305}"

for boolean_name in \
  POLICY_ARMED \
  POLICY_MOTION_ONLY \
  POLICY_SKIP_BUILD \
  POLICY_REUSE_SERVER \
  POLICY_KEEP_SERVER \
  POLICY_STOP_SERVER_ONLY \
  POLICY_INTERACTIVE; do
  case "${!boolean_name}" in
    true|false) ;;
    *) echo "ERROR: ${boolean_name} must be true or false" >&2; exit 2 ;;
  esac
done
if [[ "${POLICY_STOP_SERVER_ONLY}" != "true" \
      && "${POLICY_MOTION_ONLY}" == "true" \
      && "${POLICY_ARMED}" != "true" ]]; then
  echo "ERROR: POLICY_MOTION_ONLY requires POLICY_ARMED=true" >&2
  exit 2
fi
if [[ "${POLICY_STOP_SERVER_ONLY}" != "true" \
      && "${POLICY_ARMED}" == "true" \
      && ! -f "${POLICY_START_POSE_FILE}" ]]; then
  echo "ERROR: policy start pose not found: ${POLICY_START_POSE_FILE}" >&2
  exit 2
fi

launch_pid=""
marker_file=""
cleanup_started=0

source_setup_file() {
  local had_nounset=0
  [[ $- == *u* ]] && had_nounset=1
  set +u
  # ROS Humble setup scripts read optional environment variables unguarded.
  # shellcheck disable=SC1090
  source "$1"
  (( had_nounset )) && set -u
}

source_ros() {
  source_setup_file /opt/ros/humble/setup.bash
  if [[ -f "${ORBBEC_WS}/install/setup.bash" ]]; then
    source_setup_file "${ORBBEC_WS}/install/setup.bash"
  fi
  source_setup_file "${WORKSPACE_DIR}/install/setup.bash"
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if timeout 2s ros2 service type "${service_name}" 2>/dev/null | grep -q .; then
      return 0
    fi
    sleep 0.25
  done
  echo "ERROR: timed out waiting for ROS service ${service_name}" >&2
  return 1
}

call_service_checked() {
  local label="$1"
  shift
  local output
  if ! output="$(timeout 40s ros2 service call "$@" 2>&1)"; then
    printf '%s\n' "${output}" >&2
    echo "ERROR: ${label} service call failed" >&2
    return 1
  fi
  printf '%s\n' "${output}"
  if ! grep -Eq 'success=(True|true)' <<<"${output}"; then
    echo "ERROR: ${label} was rejected" >&2
    return 1
  fi
}

verify_gripper_ready() {
  local output
  if ! output="$(timeout 12s ros2 service call /get_gripper_state \
      dobot_interfaces/srv/GripperState "{}" 2>&1)"; then
    printf '%s\n' "${output}" >&2
    echo "ERROR: gripper state service call failed" >&2
    return 1
  fi
  printf '%s\n' "${output}"
  if ! grep -Eq 'success=(True|true)' <<<"${output}" \
      || ! grep -q 'connected=True' <<<"${output}" \
      || ! grep -q 'initialized=True' <<<"${output}" \
      || ! grep -Eq 'opening_mm=([5-9][5-9](\.[0-9]+)?|[6-9][0-9](\.[0-9]+)?|[1-9][0-9]{2,}(\.[0-9]+)?)' <<<"${output}"; then
    echo "ERROR: gripper is not connected, initialized, and open (>=55 mm)" >&2
    return 1
  fi
}

move_to_policy_start_pose() {
  local start_joints
  start_joints="$("${POLICY_PYTHON}" -c \
    'import json, sys; print(", ".join(f"{float(v):.9f}" for v in json.load(open(sys.argv[1], encoding="utf-8"))["joints_deg"]))' \
    "${POLICY_START_POSE_FILE}")"
  echo "Returning to recorded policy start pose: [${start_joints}]"
  call_service_checked "MoveJ to policy start pose" \
    /movej dobot_interfaces/srv/MoveCommand \
    "{target: [${start_joints}], user: 0, tool: 0, speed: 10, acceleration: 10, wait: true, timeout_sec: 30.0}"
}

prepare_policy_start_state() {
  call_service_checked "open gripper for policy start state" \
    /gripper_move dobot_interfaces/srv/GripperCommand \
    "{opening_mm: 95.0, position_permille: -1, force_percent: 50, force_n: -1.0, wait: true, timeout_sec: 8.0}"
  verify_gripper_ready
  move_to_policy_start_pose
  # The gripper was verified immediately before the synchronous MoveJ, which
  # cannot change its state. Give the Dobot request/reply channel time to settle
  # before switching to the 33 Hz ServoP stream; an immediate Modbus read here
  # can block while the controller changes motion modes.
  sleep 1.5
}

wait_for_policy_episode() {
  local policy_log before_finished finished deadline
  policy_log="$(find "${POLICY_LOG_DIR}" -maxdepth 1 -type f -name 'pi05_tape_grasp_*.jsonl' \
    -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)"
  before_finished=0
  [[ -n "${policy_log}" ]] && before_finished="$(grep -c '"event":"episode_finished"' "${policy_log}" || true)"
  deadline=$((SECONDS + RUN_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      echo "ERROR: ROS launch exited before the policy reported completion" >&2
      return 1
    fi
    if [[ -n "${policy_log}" ]]; then
      finished="$(grep -c '"event":"episode_finished"' "${policy_log}" || true)"
      if (( finished > before_finished )); then
        grep '"event":"episode_finished"' "${policy_log}" | tail -n 1
        return 0
      fi
    fi
    sleep 0.5
  done
  echo "ERROR: policy episode did not finish within ${RUN_TIMEOUT_SEC}s" >&2
  return 1
}

stop_policy_server() {
  docker compose \
    -f "${OPENPI_DIR}/compose.yaml" \
    -f "${OPENPI_DIR}/compose.gpu.yaml" \
    exec -T openpi sh -lc \
    "pkill -TERM -f '[s]cripts/serve_policy.py.*pi05_dobot_pen_box_servo_p_action_only' || true" \
    >/dev/null 2>&1 || true
  rm -f -- "${SERVER_META_FILE}"
}

cleanup() {
  local status="${1:-0}"
  if (( cleanup_started )); then
    return
  fi
  cleanup_started=1
  set +e
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    timeout 4s ros2 service call /dobot_policy/stop std_srvs/srv/Trigger "{}" >/dev/null 2>&1
    sleep 0.4
    if [[ "${POLICY_ARMED}" == "true" ]]; then
      timeout 4s ros2 service call /disable_robot std_srvs/srv/Trigger "{}" >/dev/null 2>&1
    fi
    kill -INT -- "-${launch_pid}" 2>/dev/null
    wait "${launch_pid}" 2>/dev/null
  fi
  if [[ "${POLICY_KEEP_SERVER}" != "true" ]]; then
    stop_policy_server
  fi
  if [[ -n "${marker_file}" ]]; then
    rm -f -- "${marker_file}"
  fi
  set -e
  exit "${status}"
}

trap 'cleanup 130' INT TERM
trap 'cleanup $?' EXIT

if [[ "${POLICY_STOP_SERVER_ONLY}" == "true" ]]; then
  command -v docker >/dev/null || {
    echo "ERROR: required command not found: docker" >&2
    exit 2
  }
  stop_policy_server
  echo "warm Policy Server stopped"
  cleanup_started=1
  exit 0
fi

for command in docker colcon curl flock; do
  command -v "${command}" >/dev/null || {
    echo "ERROR: required command not found: ${command}" >&2
    exit 2
  }
done
[[ -x "${UV_BIN}" ]] || { echo "ERROR: uv not found: ${UV_BIN}" >&2; exit 2; }
[[ -d "${OPENPI_DIR}" ]] || { echo "ERROR: OpenPI repo not found: ${OPENPI_DIR}" >&2; exit 2; }
[[ -d "${CHECKPOINT_DIR}/params" ]] || {
  echo "ERROR: final checkpoint not found: ${CHECKPOINT_DIR}" >&2
  exit 2
}

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "ERROR: another Dobot system process owns ${LOCK_FILE}" >&2
  exit 2
fi

cd "${WORKSPACE_DIR}"
mkdir -p "${POLICY_LOG_DIR}" "$(dirname "${SERVER_LOG}")"

if [[ ! -x "${POLICY_PYTHON}" ]]; then
  "${UV_BIN}" venv --python /usr/bin/python3 --system-site-packages "${WORKSPACE_DIR}/.venv-policy"
fi
if ! "${POLICY_PYTHON}" -c 'import openpi_client, websockets, msgpack' 2>/dev/null; then
  "${UV_BIN}" pip install \
    --python "${POLICY_PYTHON}" \
    -e "${OPENPI_DIR}/packages/openpi-client"
fi

source_setup_file /opt/ros/humble/setup.bash
if [[ -f "${ORBBEC_WS}/install/setup.bash" ]]; then
  source_setup_file "${ORBBEC_WS}/install/setup.bash"
fi
if [[ "${POLICY_SKIP_BUILD}" == "true" ]]; then
  [[ -x "${WORKSPACE_DIR}/install/dobot_policy/lib/dobot_policy/dobot_policy_node" ]] || {
    echo "ERROR: POLICY_SKIP_BUILD=true but dobot_policy is not installed" >&2
    exit 2
  }
  echo "Fast demo: reusing the installed ROS packages (build skipped)."
else
  colcon build --symlink-install --packages-up-to dobot_policy
fi
source_ros

docker compose \
  -f "${OPENPI_DIR}/compose.yaml" \
  -f "${OPENPI_DIR}/compose.gpu.yaml" \
  up -d openpi
server_ready=0
server_identity="${POLICY_CONFIG}|${CHECKPOINT_CONTAINER_DIR}|${POLICY_PORT}"
if [[ "${POLICY_REUSE_SERVER}" == "true" \
      && -f "${SERVER_META_FILE}" \
      && "$(<"${SERVER_META_FILE}")" == "${server_identity}" ]] \
    && curl --noproxy '*' --silent --fail \
      "http://127.0.0.1:${POLICY_PORT}/healthz" >/dev/null 2>&1; then
  server_ready=1
  echo "Fast demo: reusing the warm OpenPI Policy Server on port ${POLICY_PORT}."
else
  stop_policy_server
  sleep 0.5
  docker compose \
    -f "${OPENPI_DIR}/compose.yaml" \
    -f "${OPENPI_DIR}/compose.gpu.yaml" \
    exec -d -T openpi sh -lc \
    "uv run --no-sync scripts/serve_policy.py --port=${POLICY_PORT} policy:checkpoint \
      --policy.config=${POLICY_CONFIG} \
      --policy.dir=${CHECKPOINT_CONTAINER_DIR} \
      > /workspace/wandb/dobot_pen_box_servo_p_action_only_v1_long_deploy.log 2>&1"
  printf '%s\n' "${server_identity}" >"${SERVER_META_FILE}"
fi

echo "Waiting for OpenPI Policy Server on 127.0.0.1:${POLICY_PORT} ..."
for _ in $(seq 1 120); do
  # Docker publishes the host port before the model process is listening.
  # The policy server's health endpoint only returns once WebSocket serving is ready.
  if curl --noproxy '*' --silent --fail \
    "http://127.0.0.1:${POLICY_PORT}/healthz" >/dev/null 2>&1; then
    server_ready=1
    break
  fi
  sleep 1
done
if (( ! server_ready )); then
  tail -n 80 "${SERVER_LOG}" >&2 || true
  stop_policy_server
  echo "ERROR: Policy Server did not become ready" >&2
  exit 1
fi
if [[ "${POLICY_KEEP_SERVER}" == "true" ]]; then
  echo "The Policy Server will remain warm after this demo for faster subsequent starts."
fi

if [[ "${POLICY_MOTION_ONLY}" == "true" ]]; then
  echo
  echo "WARNING: 100% ServoP MOTION-ONLY validation starts in ${COUNTDOWN_SEC} seconds."
  echo "The gripper is disabled. The robot will first MoveJ to the recorded start pose,"
  echo "then execute policy Cartesian motion at full configured scale for ${POLICY_MOTION_TEST_DURATION_SEC}s."
  echo "Keep the emergency stop in hand and keep people clear of the workspace."
  echo "Press Ctrl-C now to abort."
  sleep "${COUNTDOWN_SEC}"
elif [[ "${POLICY_ARMED}" == "true" ]]; then
  echo
  echo "WARNING: ARMED warm policy session will start in ${COUNTDOWN_SEC} seconds."
  echo "Keep the emergency stop in hand and keep people clear of the workspace."
  echo "Task: pick the pen and put it in the box"
  echo "Press Ctrl-C now to abort; after startup, press r to run an attempt."
  sleep "${COUNTDOWN_SEC}"
else
  echo "DRY-RUN: policy inference only; robot and gripper commands are disabled."
fi

# The policy endpoint is local/LAN traffic. websockets 16 otherwise inherits
# shell proxy variables and may send ws://127.0.0.1 through the HTTP proxy.
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY="localhost,127.0.0.1,::1"
export no_proxy="${NO_PROXY}"
export ROS_LOG_DIR="${WORKSPACE_DIR}/logs/ros_policy_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${ROS_LOG_DIR}"

marker_file="$(mktemp /tmp/dobot_policy_run.XXXXXX)"
# Start the policy only after the real-hardware preflight below succeeds.
# In particular, the Dobot Modbus bridge can be unavailable briefly just after
# EnableRobot, so a policy action must never race gripper initialization.
launch_auto_start=false
launch_auto_enable=false
launch_gripper_enabled=true
if [[ "${POLICY_MOTION_ONLY}" == "true" ]]; then
  launch_auto_start=false
  launch_auto_enable=false
  launch_gripper_enabled=false
fi
# A real policy run requires a valid GripperStatus before any policy action may
# execute. Initializing here also makes the one-command workflow robust after a
# previous run disabled the robot.
setsid ros2 launch dobot_policy pi05_tape_grasp.launch.py \
  armed:="${POLICY_ARMED}" \
  auto_start:="${launch_auto_start}" \
  auto_enable_robot:="${launch_auto_enable}" \
  auto_init_gripper:=false \
  gripper_enabled:="${launch_gripper_enabled}" \
  motion_test_duration_sec:="${POLICY_MOTION_TEST_DURATION_SEC}" \
  max_episode_sec:="${POLICY_MAX_EPISODE_SEC}" \
  policy_host:=127.0.0.1 \
  policy_port:="${POLICY_PORT}" \
  policy_python:="${POLICY_PYTHON}" &
launch_pid=$!

if [[ "${POLICY_MOTION_ONLY}" == "true" ]]; then
  wait_for_service /enable_robot 30
  wait_for_service /movej 30
  wait_for_service /dobot_policy/start 30

  call_service_checked "enable robot" \
    /enable_robot std_srvs/srv/Trigger "{}"

  move_to_policy_start_pose
  # Do not switch from request/reply motion to ServoP on the same controller
  # socket immediately after Sync().
  sleep 1.5

  call_service_checked "start motion-only policy" \
    /dobot_policy/start std_srvs/srv/Trigger "{}"
elif [[ "${POLICY_ARMED}" == "true" ]]; then
  wait_for_service /enable_robot 30
  wait_for_service /movej 30
  wait_for_service /gripper_init 30
  wait_for_service /gripper_move 30
  wait_for_service /get_gripper_state 30
  wait_for_service /dobot_policy/start 30
  call_service_checked "enable robot" \
    /enable_robot std_srvs/srv/Trigger "{}"
  # Let the controller's internal Modbus bridge settle after EnableRobot.
  sleep 1.0
  call_service_checked "initialize gripper" \
    /gripper_init std_srvs/srv/Trigger "{}"
  sleep 0.5
  if [[ "${POLICY_INTERACTIVE}" == "true" ]]; then
    echo
    echo "Warm policy session ready. Press r to reset to the recorded start pose and run; q to stop."
    printf '[r] run  [q] quit > '
    while true; do
      key=""
      IFS= read -rsn1 -t 0.25 key || true
      if [[ -z "${key}" ]]; then
        continue
      fi
      echo
      case "${key}" in
        r|R)
          # Pressing r is a restart request even while an episode is active.
          # Stop only the policy episode; keep ROS, cameras, controller and
          # the already initialized gripper alive.
          timeout 4s ros2 service call /dobot_policy/stop \
            std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
          sleep 0.3
          prepare_policy_start_state
          call_service_checked "start policy after start-state preflight" \
            /dobot_policy/start std_srvs/srv/Trigger "{}"
          echo "Policy episode running. Press r to interrupt and restart, or q to stop."
          printf '[r] run  [q] quit > '
          ;;
        q|Q)
          cleanup 0
          ;;
        *)
          echo "Use r to run or q to stop."
          printf '[r] run  [q] quit > '
          ;;
      esac
    done
  fi
  prepare_policy_start_state
  call_service_checked "start policy after start-state preflight" \
    /dobot_policy/start std_srvs/srv/Trigger "{}"
else
  wait_for_service /dobot_policy/start 30
  call_service_checked "start dry-run policy" \
    /dobot_policy/start std_srvs/srv/Trigger "{}"
fi

deadline=$((SECONDS + RUN_TIMEOUT_SEC))
result_status=1
while (( SECONDS < deadline )); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "ERROR: ROS launch exited before the policy reported completion" >&2
    break
  fi
  latest_log="$(
    find "${POLICY_LOG_DIR}" -maxdepth 1 -type f -name 'pi05_tape_grasp_*.jsonl' \
      -newer "${marker_file}" -printf '%T@ %p\n' 2>/dev/null \
      | sort -n | tail -n 1 | cut -d' ' -f2-
  )"
  if [[ -n "${latest_log}" ]] && grep -q '"event":"episode_finished"' "${latest_log}"; then
    if grep '"event":"episode_finished"' "${latest_log}" | tail -n 1 | grep -q '"success":true'; then
      if [[ "${POLICY_MOTION_ONLY}" == "true" ]]; then
        echo "Policy reported 100% ServoP motion-only validation success. Log: ${latest_log}"
      else
        echo "Policy reported grasp success. Log: ${latest_log}"
      fi
      result_status=0
    else
      echo "Policy stopped without grasp success. Log: ${latest_log}" >&2
    fi
    break
  fi
  if [[ "${POLICY_ARMED}" == "false" ]] \
    && [[ -n "${latest_log}" ]] \
    && grep -q '"event":"inference_complete"' "${latest_log}"; then
    echo "Dry-run inference completed. Log: ${latest_log}"
    result_status=0
    break
  fi
  sleep 0.5
done
if (( SECONDS >= deadline )); then
  echo "ERROR: real-robot verification timed out" >&2
fi

cleanup "${result_status}"

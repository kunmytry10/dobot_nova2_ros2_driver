const state = {
  trajectories: [],
  joints: [],
  tcp: [],
  lastPayload: {},
};

const $ = (id) => document.getElementById(id);

async function apiGet(path) {
  const response = await fetch(path, { cache: "no-store" });
  return response.json();
}

async function apiPost(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  return response.json();
}

function logResult(title, payload) {
  const line = `[${new Date().toLocaleTimeString()}] ${title}\n${JSON.stringify(payload, null, 2)}\n\n`;
  $("resultLog").textContent = line + $("resultLog").textContent;
}

function formatNumber(value, digits = 3) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "-";
}

function formatTargetValues(values) {
  return values
    .slice(0, 6)
    .map((value) => formatNumber(value))
    .join(", ");
}

function hasCompleteValues(values) {
  return values.length >= 6 && values.slice(0, 6).every((value) => Number.isFinite(Number(value)));
}

function numberValues(values) {
  if (Array.isArray(values)) {
    return values.map((value) => Number(value)).filter((value) => Number.isFinite(value));
  }
  if (typeof values === "string") {
    const matches = values.match(/[-+]?(?:\d+\.?\d*|\.\d+)(?:e[-+]?\d+)?/gi);
    return (matches || []).map((value) => Number(value)).filter((value) => Number.isFinite(value));
  }
  return [];
}

function sixNumberValues(...sources) {
  for (const source of sources) {
    const values = numberValues(source);
    if (values.length >= 6) {
      return values.slice(0, 6);
    }
  }
  return [];
}

function radiansToDegrees(values) {
  const numbers = numberValues(values);
  return numbers.map((value) => value * 180 / Math.PI);
}

function setMoveTarget(text) {
  const input = $("moveTarget");
  input.value = text;
  input.focus({ preventScroll: true });
  input.select();
}

function renderMetrics(containerId, names, values, unit) {
  const container = $(containerId);
  container.innerHTML = "";
  names.forEach((name, index) => {
    const value = Number(values[index]);
    const displayUnit = Array.isArray(unit) ? unit[index] : unit;
    const row = document.createElement("div");
    row.className = "metric-row";
    row.dataset.value = Number.isFinite(value) ? String(value) : "";
    row.innerHTML = `<span>${name}</span><strong>${formatNumber(value)} ${displayUnit}</strong>`;
    container.appendChild(row);
  });
}

function renderState(payload) {
  state.lastPayload = payload || {};
  const robot = payload.robot_state || {};
  const cachedStatus = payload.cached?.dobot_state || {};
  const joint = payload.joint_state || {};
  const tcp = payload.tcp_pose || {};
  const gripper = payload.gripper_state || payload.cached?.gripper_state || {};

  const connected = robot.connected ? "connected" : "not connected";
  const mode = robot.robot_mode_text || cachedStatus.robot_mode || "-";
  $("connectionLine").textContent = `${connected} | services ${readyCount(payload.services)}`;
  $("robotMode").textContent = `${robot.robot_mode ?? "-"} ${mode}`;
  $("enableStatus").textContent = valueOrCached(robot.enable_status, cachedStatus.enable_status);
  $("runningStatus").textContent = valueOrCached(robot.running_status, cachedStatus.running_status);
  $("errorStatus").textContent = valueOrCached(robot.error_status, cachedStatus.error_status);
  $("dragStatus").textContent = cachedStatus.drag_status ?? "-";
  $("speedScaling").textContent = formatNumber(robot.speed_scaling ?? cachedStatus.speed_scaling, 2);

  const cachedJoint = payload.cached?.joint_state || {};
  const joints = sixNumberValues(
    joint.joints_deg,
    radiansToDegrees(joint.joints_rad),
    radiansToDegrees(cachedJoint.position),
  );
  state.joints = joints;
  renderMetrics("jointList", ["J1", "J2", "J3", "J4", "J5", "J6"], joints, "deg");

  const pose = sixNumberValues(tcp.pose, payload.cached?.tcp_pose);
  state.tcp = pose;
  renderMetrics("tcpList", ["X", "Y", "Z", "Rx", "Ry", "Rz"], pose, [
    "mm",
    "mm",
    "mm",
    "deg",
    "deg",
    "deg",
  ]);
  renderGripper(gripper);
}

function renderGripper(gripper) {
  $("gripperInit").textContent = `${gripper.init_state ?? "-"} ${gripper.initialized ? "initialized" : ""}`.trim();
  $("gripperGrip").textContent = `${gripper.grip_state ?? "-"} ${gripStateText(gripper.grip_state)}`;
  $("gripperOpening").textContent = `${formatNumber(gripper.opening_mm)} mm`;
  $("gripperForce").textContent = `${valueOrCached(gripper.force_percent, "-")} %`;
  $("gripperObject").textContent = gripper.object_detected
    ? "detected"
    : gripper.object_dropped
      ? "dropped"
      : "-";
  $("gripperConnected").textContent = gripper.connected ? "connected" : "not connected";
}

function gripStateText(value) {
  if (value === 0) return "moving";
  if (value === 1) return "reached";
  if (value === 2) return "object";
  if (value === 3) return "dropped";
  return "";
}

function valueOrCached(value, cached) {
  return value !== undefined && value !== null ? value : cached ?? "-";
}

function readyCount(services) {
  if (!services) return "0/0";
  const values = Object.values(services);
  return `${values.filter(Boolean).length}/${values.length}`;
}

async function refreshState() {
  try {
    renderState(await apiGet("/api/state"));
  } catch (error) {
    logResult("state refresh failed", { message: String(error) });
  }
}

async function refreshTrajectories() {
  const result = await apiGet("/api/trajectories");
  state.trajectories = (result.names || []).map((name, index) => ({
    name,
    path: result.paths?.[index] || "",
    point_count: result.point_counts?.[index] || 0,
  }));
  renderTrajectories();
}

function renderTrajectories() {
  const container = $("trajectoryList");
  container.innerHTML = "";
  if (!state.trajectories.length) {
    container.textContent = "No trajectories saved.";
    return;
  }
  state.trajectories.forEach((trajectory) => {
    const row = document.createElement("div");
    row.className = "trajectory-row";
    row.innerHTML = `
      <button type="button">${trajectory.name}</button>
      <strong>${trajectory.point_count} points</strong>
      <span>${trajectory.path}</span>
    `;
    row.querySelector("button").addEventListener("click", () => {
      $("trajectoryName").value = trajectory.name;
    });
    container.appendChild(row);
  });
}

function parseTarget() {
  const values = $("moveTarget").value
    .split(",")
    .map((value) => Number(value.trim()))
    .filter((value) => Number.isFinite(value));
  while (values.length < 6) values.push(0);
  return values.slice(0, 6);
}

function displayedMetricValues(containerId) {
  const rows = Array.from($(containerId).querySelectorAll(".metric-row"));
  return rows
    .map((row) => {
      const dataValues = numberValues(row.dataset.value);
      if (dataValues.length) {
        return dataValues[0];
      }
      const textValues = numberValues(row.querySelector("strong")?.textContent || "");
      return textValues.length ? textValues[0] : Number.NaN;
    })
    .filter((value) => Number.isFinite(value));
}

function copySourceValues(kind) {
  if (kind === "joints") {
    return sixNumberValues(state.joints, displayedMetricValues("jointList"));
  }
  return sixNumberValues(state.tcp, displayedMetricValues("tcpList"));
}

function copyDebug(kind) {
  const payload = state.lastPayload || {};
  const joint = payload.joint_state || {};
  const tcp = payload.tcp_pose || {};
  const cachedJoint = payload.cached?.joint_state || {};
  return {
    kind,
    state_values: kind === "joints" ? state.joints.length : state.tcp.length,
    dom_values: displayedMetricValues(kind === "joints" ? "jointList" : "tcpList").length,
    joint_ok: joint.ok,
    joint_success: joint.success,
    joint_message: joint.message,
    joint_fields: Object.keys(joint),
    tcp_ok: tcp.ok,
    tcp_success: tcp.success,
    tcp_message: tcp.message,
    tcp_fields: Object.keys(tcp),
    cached_joint_position_values: numberValues(cachedJoint.position).length,
    cached_tcp_values: numberValues(payload.cached?.tcp_pose).length,
    app_version: "copy-fallback-v2",
  };
}

async function copyValues(kind, button) {
  const values = copySourceValues(kind);
  const text = formatTargetValues(values);
  if (!hasCompleteValues(values)) {
    logResult(`copy ${kind}`, {
      ok: false,
      message: "no complete state values available",
      debug: copyDebug(kind),
    });
    return;
  }

  setMoveTarget(text);

  let copied = false;
  let message = "target filled; clipboard copy not available";
  try {
    copied = document.execCommand("copy");
  } catch (error) {
    message = String(error);
  }

  if (!copied && navigator.clipboard && window.isSecureContext) {
    try {
      await navigator.clipboard.writeText(text);
      copied = true;
    } catch (error) {
      message = String(error);
    }
  }

  const original = button.textContent;
  button.textContent = copied ? "Copied" : "Filled";
  window.setTimeout(() => {
    button.textContent = original;
  }, 900);
  logResult(`copy ${kind}`, { ok: true, copied, message, value: text });
}

async function callTrigger(service) {
  const result = await apiPost("/api/trigger", { service });
  logResult(service, result);
  await refreshState();
}

async function sendMove() {
  const payload = {
    kind: $("moveKind").value,
    target: parseTarget(),
    speed: Number($("moveSpeed").value || 0),
    acceleration: Number($("moveAcc").value || 0),
    wait: $("moveWait").checked,
    timeout_sec: Number($("moveTimeout").value || 0),
  };
  const result = await apiPost("/api/move", payload);
  logResult(`move ${payload.kind}`, result);
  await refreshState();
}

function gripperPayload(openingMm) {
  return {
    opening_mm: openingMm,
    position_permille: 1000,
    force_percent: Number($("gripperForceInput").value || 0),
    force_n: Number($("gripperForceNInput").value || -1),
    wait: $("gripperWait").checked,
    timeout_sec: Number($("gripperTimeout").value || 0),
  };
}

async function gripperMove(openingMm, label) {
  const result = await apiPost("/api/gripper/move", gripperPayload(openingMm));
  logResult(label, result);
  await refreshState();
}

function teachPayload() {
  return {
    name: $("trajectoryName").value.trim(),
    overwrite: $("overwriteTrajectory").checked,
    speed: Number($("teachSpeed").value || 0),
    acceleration: Number($("teachAcc").value || 0),
    replay_mode: $("replayMode").value,
    override_wait: true,
    wait: true,
    timeout_sec: Number($("teachTimeout").value || 0),
  };
}

async function teach(action) {
  const result = await apiPost(`/api/teach/${action}`, teachPayload());
  logResult(`teach ${action}`, result);
  await refreshState();
  if (["stop", "delete", "replay"].includes(action)) {
    await refreshTrajectories();
  }
}

function bindEvents() {
  document.querySelectorAll("[data-trigger]").forEach((button) => {
    button.addEventListener("click", () => callTrigger(button.dataset.trigger));
  });
  $("emergencyStop").addEventListener("click", () => callTrigger("emergency_stop"));
  $("copyJoints").addEventListener("click", (event) => copyValues("joints", event.currentTarget));
  $("copyTcp").addEventListener("click", (event) => copyValues("tcp", event.currentTarget));
  $("sendMove").addEventListener("click", sendMove);
  $("gripperInitButton").addEventListener("click", () => callTrigger("gripper_init"));
  $("gripperOpen").addEventListener("click", () => gripperMove(95, "gripper open"));
  $("gripperClose").addEventListener("click", () => gripperMove(0, "gripper close"));
  $("gripperMove").addEventListener("click", () => {
    gripperMove(Number($("gripperOpeningInput").value || 0), "gripper move");
  });
  $("teachStart").addEventListener("click", () => teach("start"));
  $("teachStop").addEventListener("click", () => teach("stop"));
  $("teachReplay").addEventListener("click", () => teach("replay"));
  $("teachDelete").addEventListener("click", () => teach("delete"));
  $("refreshTrajectories").addEventListener("click", refreshTrajectories);
}

bindEvents();
refreshState();
refreshTrajectories();
setInterval(refreshState, 1000);

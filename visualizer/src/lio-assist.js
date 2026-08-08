import {
  Activity,
  Cpu,
  createIcons,
  LocateFixed,
  Play,
  Radio,
  RotateCcw,
  ShieldCheck,
  Square,
} from "lucide";
import "./lio-assist.css";

const icons = {
  Activity,
  Cpu,
  LocateFixed,
  Play,
  Radio,
  RotateCcw,
  ShieldCheck,
  Square,
};

createIcons({ icons });

const canvas = document.querySelector("#pathCanvas");
const context = canvas.getContext("2d");
const elements = {
  assistTitle: document.querySelector("#assistTitle"),
  phaseKicker: document.querySelector("#phaseKicker"),
  phaseLabel: document.querySelector("#phaseLabel"),
  phaseInstruction: document.querySelector("#phaseInstruction"),
  phaseRemaining: document.querySelector("#phaseRemaining"),
  timelineFill: document.querySelector("#timelineFill"),
  timelineTrack: document.querySelector(".timeline-track"),
  timelineLabels: document.querySelector(".timeline-labels"),
  missionTime: document.querySelector("#missionTime"),
  sessionName: document.querySelector("#sessionName"),
  primaryMetricLabel: document.querySelector("#primaryMetricLabel"),
  secondaryMetricLabel: document.querySelector("#secondaryMetricLabel"),
  distanceValue: document.querySelector("#distanceValue"),
  returnValue: document.querySelector("#returnValue"),
  positionReadout: document.querySelector("#positionReadout"),
  imuDetail: document.querySelector("#imuDetail"),
  lidarDetail: document.querySelector("#lidarDetail"),
  lioDetail: document.querySelector("#lioDetail"),
  cubeDetail: document.querySelector("#cubeDetail"),
  restartButton: document.querySelector("#restartButton"),
  stopButton: document.querySelector("#stopButton"),
  connectionStatus: document.querySelector("#connectionStatus"),
};

let latestState = null;
let connected = false;
let timelineSignature = "";

function renderTimeline(state) {
  const phases = Array.isArray(state.guide_phases) ? state.guide_phases : [];
  const signature = JSON.stringify(phases);
  if (!phases.length || signature === timelineSignature) return;
  timelineSignature = signature;

  elements.timelineTrack
    .querySelectorAll(".timeline-mark")
    .forEach((mark) => mark.remove());
  elements.timelineLabels.replaceChildren();
  const duration = Number(state.duration_s) || 1;
  elements.timelineLabels.style.gridTemplateColumns = phases
    .map((phase) => `${Math.max(0.1, phase.end_s - phase.start_s)}fr`)
    .join(" ");
  phases.forEach((phase, index) => {
    const label = document.createElement("span");
    label.textContent = phase.timeline_label || phase.label || phase.id;
    elements.timelineLabels.append(label);
    if (index < phases.length - 1) {
      const mark = document.createElement("span");
      mark.className = "timeline-mark";
      mark.style.left = `${(Number(phase.end_s) / duration) * 100}%`;
      elements.timelineTrack.append(mark);
    }
  });
}

function formatClock(seconds) {
  if (!Number.isFinite(seconds)) return "--";
  const rounded = Math.max(0, Math.ceil(seconds));
  const minutes = Math.floor(rounded / 60);
  const remainder = rounded % 60;
  return `${String(minutes).padStart(2, "0")}:${String(remainder).padStart(2, "0")}`;
}

function formatNumber(value, digits = 2, suffix = "") {
  return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "--";
}

function formatSigned(value, digits = 1, suffix = "") {
  if (!Number.isFinite(value)) return "--";
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(digits)}${suffix}`;
}

function setHealth(name, healthy, warning = false) {
  const row = document.querySelector(`[data-sensor="${name}"]`);
  row.classList.toggle("is-healthy", Boolean(healthy));
  row.classList.toggle("is-warning", !healthy && Boolean(warning));
}

function drawGrid(width, height) {
  context.fillStyle = "#111516";
  context.fillRect(0, 0, width, height);
  context.strokeStyle = "#252b2c";
  context.lineWidth = 1;
  const spacing = 44;
  for (let x = width / 2; x < width; x += spacing) {
    context.beginPath();
    context.moveTo(x, 0);
    context.lineTo(x, height);
    context.stroke();
  }
  for (let x = width / 2 - spacing; x >= 0; x -= spacing) {
    context.beginPath();
    context.moveTo(x, 0);
    context.lineTo(x, height);
    context.stroke();
  }
  for (let y = height / 2; y < height; y += spacing) {
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
  for (let y = height / 2 - spacing; y >= 0; y -= spacing) {
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
}

function drawPath(state) {
  const rect = canvas.getBoundingClientRect();
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * pixelRatio));
  const height = Math.max(1, Math.round(rect.height * pixelRatio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  drawGrid(rect.width, rect.height);

  const path = Array.isArray(state?.path) ? state.path : [];
  const position = state?.position_m;
  const coordinates = [...path];
  if (Array.isArray(position)) coordinates.push(position);
  let extent = 1.5;
  for (const point of coordinates) {
    extent = Math.max(extent, Math.abs(point[0]), Math.abs(point[1]));
  }
  extent *= 1.18;
  const scale = Math.min(rect.width, rect.height) / (2 * extent);
  const centerX = rect.width / 2;
  const centerY = rect.height / 2;
  const project = (point) => [
    centerX + point[1] * scale,
    centerY - point[0] * scale,
  ];

  context.strokeStyle = "#506061";
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(centerX, 12);
  context.lineTo(centerX, rect.height - 12);
  context.moveTo(12, centerY);
  context.lineTo(rect.width - 12, centerY);
  context.stroke();

  context.fillStyle = "#7f8c8d";
  context.font = "700 10px ui-monospace, monospace";
  context.fillText("+X", centerX + 7, 18);
  context.fillText("+Y", rect.width - 28, centerY - 7);

  context.strokeStyle = "rgba(108, 225, 154, 0.45)";
  context.lineWidth = 2;
  context.beginPath();
  context.arc(centerX, centerY, 0.35 * scale, 0, Math.PI * 2);
  context.stroke();

  if (path.length > 1) {
    context.strokeStyle = "#37c8d6";
    context.lineWidth = 3;
    context.lineJoin = "round";
    context.lineCap = "round";
    context.beginPath();
    path.forEach((point, index) => {
      const [x, y] = project(point);
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
  }

  context.fillStyle = "#6ce19a";
  context.fillRect(centerX - 5, centerY - 5, 10, 10);

  if (Array.isArray(position)) {
    const [x, y] = project(position);
    context.fillStyle = "#ffbf4b";
    context.beginPath();
    context.arc(x, y, 7, 0, Math.PI * 2);
    context.fill();
    context.strokeStyle = "#101314";
    context.lineWidth = 2;
    context.stroke();
  }
}

function render(state) {
  latestState = state;
  renderTimeline(state);
  elements.assistTitle.textContent =
    state.guide_kind === "yaw"
      ? "Repeated yaw validation"
      : state.guide_kind === "translation"
        ? "Translation scale validation"
        : "Guided carry validation";
  const phase = state.phase || {};
  document
    .querySelector(".phase-band")
    .classList.toggle("is-failed", Boolean(state.failed));
  elements.phaseKicker.textContent = state.guide_started
    ? `PHASE ${String(phase.id || "").replaceAll("_", " ").toUpperCase()}`
    : "WAITING FOR CLOCK LOCK";
  elements.phaseLabel.textContent = phase.label || "SYNCHRONIZING";
  elements.phaseInstruction.textContent = phase.instruction || "";
  elements.phaseRemaining.textContent = formatClock(phase.remaining_s);
  elements.timelineFill.style.width = `${Math.max(0, Math.min(100, state.progress * 100))}%`;
  elements.missionTime.textContent = formatClock(state.elapsed_s);
  elements.sessionName.textContent = state.session || "SESSION STARTING";
  const yawGuide = state.guide_kind === "yaw";
  const translationGuide = state.guide_kind === "translation";
  if (yawGuide) {
    const yaw = state.yaw || {};
    elements.primaryMetricLabel.textContent = "CUBE YAW";
    elements.secondaryMetricLabel.textContent = "YAW RATE";
    elements.distanceValue.textContent = formatSigned(
      yaw.cube_delta_deg,
      1,
      " deg",
    );
    elements.returnValue.textContent = formatSigned(
      yaw.cube_rate_dps,
      1,
      " deg/s",
    );
    elements.positionReadout.textContent =
      `LIO ${formatSigned(yaw.lio_delta_deg, 1, " DEG")} / ` +
      `CUBE ${formatSigned(yaw.cube_delta_deg, 1, " DEG")} / ` +
      `PEAK ${formatNumber(yaw.cube_maximum_rate_dps, 1, " DEG/S")}`;
  } else if (translationGuide) {
    const translation = state.translation || {};
    const body = Array.isArray(translation.cube_body_delta_m)
      ? translation.cube_body_delta_m
      : [];
    elements.primaryMetricLabel.textContent = "CUBE FORWARD";
    elements.secondaryMetricLabel.textContent = "CUBE RIGHT";
    elements.distanceValue.textContent = formatSigned(body[0], 2, " m");
    elements.returnValue.textContent = formatSigned(body[1], 2, " m");
    const lio = Array.isArray(state.position_m) ? state.position_m : [];
    elements.positionReadout.textContent =
      `LIO X ${formatSigned(lio[0], 2, " M")} / ` +
      `Y ${formatSigned(lio[1], 2, " M")} / ` +
      `SPEED ${formatNumber(translation.cube_horizontal_speed_mps, 2, " M/S")} / ` +
      `YAW ${formatNumber(translation.cube_maximum_yaw_deviation_deg, 1, " DEG")}`;
  } else if (Array.isArray(state.position_m)) {
    elements.primaryMetricLabel.textContent = "DISTANCE";
    elements.secondaryMetricLabel.textContent = "RETURN ERROR";
    elements.distanceValue.textContent = formatNumber(state.distance_m, 2, " m");
    elements.returnValue.textContent = formatNumber(state.return_error_m, 2, " m");
    const [x, y, z] = state.position_m;
    elements.positionReadout.textContent =
      `X ${x.toFixed(2)} / Y ${y.toFixed(2)} / Z ${z.toFixed(2)} M`;
  } else {
    elements.primaryMetricLabel.textContent = "DISTANCE";
    elements.secondaryMetricLabel.textContent = "RETURN ERROR";
    elements.distanceValue.textContent = formatNumber(state.distance_m, 2, " m");
    elements.returnValue.textContent = formatNumber(state.return_error_m, 2, " m");
    elements.positionReadout.textContent = "X -- / Y -- / Z --";
  }

  const imuHealthy =
    state.imu.connected &&
    state.imu.clock_ready &&
    state.imu.queue_drops === 0 &&
    Number(state.imu.rate_hz) >= 180;
  elements.imuDetail.textContent =
    `${formatNumber(state.imu.rate_hz, 1, " Hz")} / ` +
    `${formatNumber(state.imu.clock_p95_ms, 1, " ms")}`;
  setHealth("imu", imuHealthy, state.imu.connected);

  const lidarHealthy =
    state.lidar.connected &&
    state.lidar.clock_ready &&
    state.lidar.queue_drops === 0 &&
    Number(state.lidar.rate_hz) >= 4;
  elements.lidarDetail.textContent =
    `${formatNumber(state.lidar.rate_hz, 1, " Hz")} / ` +
    `${formatNumber(state.lidar.clock_p95_ms, 1, " ms")}`;
  setHealth("lidar", lidarHealthy, state.lidar.connected);

  elements.lioDetail.textContent =
    `${state.odometry_rows} POSES / ${state.synchronized ? "SYNCED" : "WAITING"}`;
  setHealth("lio", state.synchronized && state.publishing);

  elements.cubeDetail.textContent =
    state.guide_kind === "yaw"
      ? `${formatNumber(state.cube_attitude_age_ms, 0, " ms")} / ATTITUDE`
      : state.guide_kind === "translation"
        ? `${formatNumber(state.cube_local_position_age_ms, 0, " ms")} / LOCAL`
      : `${state.cube_local_position_rows} LOCAL / ${state.cube_messages} TOTAL`;
  setHealth(
    "cube",
    state.guide_kind === "yaw"
      ? state.cube_attitude_fresh
      : state.guide_kind === "translation"
        ? state.cube_attitude_fresh && state.cube_local_position_fresh
      : state.cube_messages > 0,
    state.cube_messages > 0,
  );

  elements.restartButton.disabled = !state.ready || state.failed;
  elements.restartButton.innerHTML = state.guide_started
    ? '<i data-lucide="rotate-ccw"></i><span>Restart sequence</span>'
    : '<i data-lucide="play"></i><span>Start sequence</span>';
  createIcons({ icons });

  elements.connectionStatus.textContent = state.failed
    ? "SAFETY STOP / NO POSE SENT TO CUBE"
    : state.ready
      ? "LIVE SHADOW DATA / NO CUBE OUTPUT"
      : "SHADOW PIPELINE SYNCHRONIZING";
  elements.connectionStatus.classList.toggle(
    "is-live",
    state.ready && !state.failed,
  );
  drawPath(state);
}

async function post(path) {
  const response = await fetch(path, { method: "POST" });
  if (!response.ok) {
    const payload = await response.json();
    throw new Error(payload.detail || `Request failed: ${response.status}`);
  }
  return response.json();
}

elements.restartButton.addEventListener("click", async () => {
  try {
    await post("/api/lio-assist/start");
  } catch (error) {
    elements.connectionStatus.textContent = String(error.message || error);
    elements.connectionStatus.classList.remove("is-live");
  }
});

elements.stopButton.addEventListener("click", async () => {
  if (!window.confirm("Stop the shadow recording and finalize its report?")) {
    return;
  }
  elements.stopButton.disabled = true;
  await post("/api/lio-assist/stop");
  elements.connectionStatus.textContent = "FINALIZING SHADOW REPORT";
});

async function poll() {
  try {
    const response = await fetch("/api/lio-assist", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    connected = true;
    render(state);
  } catch {
    if (connected) {
      elements.connectionStatus.textContent = "SHADOW RECORDER DISCONNECTED";
      elements.connectionStatus.classList.remove("is-live");
    }
    connected = false;
  }
}

window.addEventListener("resize", () => drawPath(latestState));
poll();
setInterval(poll, 200);

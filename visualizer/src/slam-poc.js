import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import {
  Activity,
  Camera,
  createIcons,
  LocateFixed,
  Radio,
  RotateCcw,
  Route,
  Scan,
  ShieldCheck,
  Square,
} from "lucide";
import { buildDrone } from "./drone.js";
import "./slam-poc.css";

createIcons({
  icons: {
    Activity,
    Camera,
    LocateFixed,
    Radio,
    RotateCcw,
    Route,
    Scan,
    ShieldCheck,
    Square,
  },
});

const elements = {
  canvas: document.querySelector("#slamScene"),
  proofStatus: document.querySelector("#proofStatus"),
  sessionName: document.querySelector("#sessionName"),
  elapsedTime: document.querySelector("#elapsedTime"),
  connectionState: document.querySelector("#connectionState"),
  mapPoints: document.querySelector("#mapPoints"),
  keyframes: document.querySelector("#keyframes"),
  trackRatio: document.querySelector("#trackRatio"),
  priorRatio: document.querySelector("#priorRatio"),
  rgbdDetail: document.querySelector("#rgbdDetail"),
  imuDetail: document.querySelector("#imuDetail"),
  lidarDetail: document.querySelector("#lidarDetail"),
  lioDetail: document.querySelector("#lioDetail"),
  rgbdPath: document.querySelector("#rgbdPath"),
  lioPath: document.querySelector("#lioPath"),
  computeTime: document.querySelector("#computeTime"),
  depthValid: document.querySelector("#depthValid"),
  rtlState: document.querySelector("#rtlState"),
  rtlCommand: document.querySelector("#rtlCommand"),
  resetView: document.querySelector("#resetView"),
  topView: document.querySelector("#topView"),
  followView: document.querySelector("#followView"),
  stopProof: document.querySelector("#stopProof"),
  guideBanner: document.querySelector("#guideBanner"),
  guideLabel: document.querySelector("#guideLabel"),
  guideInstruction: document.querySelector("#guideInstruction"),
  guideDetail: document.querySelector("#guideDetail"),
  guideValue: document.querySelector("#guideValue"),
  guideEstimate: document.querySelector("#guideEstimate"),
  guideProgress: document.querySelector("#guideProgress"),
  instructionFlash: document.querySelector("#instructionFlash"),
};

const renderer = new THREE.WebGLRenderer({
  canvas: elements.canvas,
  antialias: true,
  powerPreference: "high-performance",
});
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.setClearColor(0x0c1011, 1);
const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x0c1011, 0.018);
const camera = new THREE.PerspectiveCamera(48, 1, 0.03, 120);
const controls = new OrbitControls(camera, elements.canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.minDistance = 0.6;
controls.maxDistance = 50;

scene.add(new THREE.HemisphereLight(0xe5efec, 0x171c1d, 1.7));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.0);
keyLight.position.set(7, 11, 5);
scene.add(keyLight);

const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(50, 50),
  new THREE.MeshStandardMaterial({ color: 0x111617, roughness: 0.95 }),
);
floor.rotation.x = -Math.PI / 2;
floor.position.y = -0.02;
scene.add(floor);
const grid = new THREE.GridHelper(50, 100, 0x435051, 0x242c2d);
grid.material.transparent = true;
grid.material.opacity = 0.62;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.8));

const drone = buildDrone();
drone.scale.setScalar(0.55);
scene.add(drone);

const rgbdLine = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0x31cad7 }),
);
const lioLine = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0xf2b84b }),
);
scene.add(rgbdLine, lioLine);
const rtlArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(),
  0.5,
  0xe96ad8,
  0.12,
  0.07,
);
rtlArrow.visible = false;
scene.add(rtlArrow);

let mapCloud = null;
let latestState = null;
let mapSequence = -1;
let following = true;
let connected = false;
let displayedGuideRevision = null;
let pendingGuide = null;
let guideFlashActive = false;
let guideFlashTimeout = null;

function scenePoint(point) {
  return new THREE.Vector3(-point[1], point[2], -point[0]);
}

function resetCamera() {
  camera.up.set(0, 1, 0);
  camera.position.set(4.8, 3.4, 5.8);
  controls.target.set(0, 0.25, 0);
  controls.enableRotate = true;
  controls.update();
  elements.topView.classList.remove("is-active");
}

function setTopView() {
  const center = drone.position;
  camera.up.set(0, 0, -1);
  camera.position.set(center.x + 0.001, center.y + 12, center.z + 0.001);
  controls.target.copy(center);
  controls.enableRotate = false;
  controls.update();
  elements.topView.classList.add("is-active");
}

function updatePath(line, values) {
  const points = Array.isArray(values) ? values.map(scenePoint) : [];
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points);
}

function decodeBase64(value) {
  const binary = atob(value);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }
  return bytes;
}

function updateMap(payload) {
  if (payload.encoding !== "int16_le_base64") return;
  const count = Number(payload.point_count) || 0;
  const pointBytes = decodeBase64(payload.points_b64 || "");
  const colorBytes = decodeBase64(payload.colors_b64 || "");
  if (pointBytes.byteLength !== count * 6 || colorBytes.byteLength !== count * 3) return;
  const view = new DataView(pointBytes.buffer, pointBytes.byteOffset, pointBytes.byteLength);
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  const scale = Number(payload.scale_m) || 0.01;
  for (let index = 0; index < count; index += 1) {
    const offset = index * 6;
    const forward = view.getInt16(offset, true) * scale;
    const left = view.getInt16(offset + 2, true) * scale;
    const up = view.getInt16(offset + 4, true) * scale;
    positions[index * 3] = -left;
    positions[index * 3 + 1] = up;
    positions[index * 3 + 2] = -forward;
    colors[index * 3] = colorBytes[index * 3] / 255;
    colors[index * 3 + 1] = colorBytes[index * 3 + 1] / 255;
    colors[index * 3 + 2] = colorBytes[index * 3 + 2] / 255;
  }
  if (mapCloud !== null) {
    scene.remove(mapCloud);
    mapCloud.geometry.dispose();
    mapCloud.material.dispose();
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  mapCloud = new THREE.Points(
    geometry,
    new THREE.PointsMaterial({ size: 0.026, vertexColors: true, sizeAttenuation: true }),
  );
  scene.add(mapCloud);
}

function percent(value) {
  return `${Math.round((Number(value) || 0) * 100)}%`;
}

function number(value, digits = 1, suffix = "") {
  return Number.isFinite(Number(value)) ? `${Number(value).toFixed(digits)}${suffix}` : "--";
}

function clock(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0));
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${String(total % 60).padStart(2, "0")}`;
}

function health(source, live, warning = false) {
  const row = document.querySelector(`[data-source="${source}"]`);
  row.classList.toggle("is-live", Boolean(live));
  row.classList.toggle("is-warning", !live && Boolean(warning));
}

function renderRtlShadow(rtl) {
  const state = String(rtl?.state || "waiting");
  const latest = rtl?.latest || {};
  elements.rtlState.textContent = state.replaceAll("_", " ").toUpperCase();
  elements.rtlCommand.textContent = number(latest.proposed_speed_mps, 2, " m/s");
  const position = latest.position_local_flu_m;
  const target = latest.target_local_flu_m;
  if (
    state !== "returning" ||
    !Array.isArray(position) ||
    !Array.isArray(target)
  ) {
    rtlArrow.visible = false;
    return;
  }
  const origin = scenePoint(position);
  const direction = scenePoint(target).sub(origin);
  const length = direction.length();
  if (length < 0.02) {
    rtlArrow.visible = false;
    return;
  }
  rtlArrow.position.copy(origin);
  rtlArrow.setDirection(direction.normalize());
  rtlArrow.setLength(Math.max(0.18, length), 0.12, 0.07);
  rtlArrow.visible = true;
}

function guideReadout(guide) {
  const phase = guide.phase || "synchronizing";
  const progress = Math.max(0, Math.min(1, Number(guide.progress) || 0));
  const rgbd = Number(guide.rgbd_horizontal_m) || 0;
  const lio = Number(guide.lio_horizontal_m) || 0;
  const estimate = `VO ${rgbd.toFixed(2)} / LIO ${lio.toFixed(2)} m`;
  if (["settle", "hold_out", "final_hold"].includes(phase)) {
    return {
      value: `${Math.max(0, Number(guide.hold_remaining_s) || 0).toFixed(1)} s`,
      estimate: "STILLNESS TIMER",
      progress,
    };
  }
  if (phase === "outbound") {
    const target = Number(guide.target_m) || 0.3;
    return {
      value: `${Math.min(rgbd, lio).toFixed(2)} / ${target.toFixed(2)} m`,
      estimate,
      progress,
    };
  }
  if (phase === "return") {
    return {
      value: `${Math.max(rgbd, lio).toFixed(2)} m left`,
      estimate,
      progress,
    };
  }
  if (phase === "complete") {
    return { value: "READY", estimate: "SAVE PROOF", progress: 1 };
  }
  return {
    value: `${Math.round(progress * 100)}%`,
    estimate: "SENSOR LOCK",
    progress,
  };
}

function commitGuide(guide, revision) {
  const readout = guideReadout(guide);
  elements.guideBanner.dataset.phase = guide.phase || "synchronizing";
  elements.guideBanner.classList.toggle(
    "is-warning",
    Boolean(guide.vertical_warning) || Number(guide.estimator_gap_m) > 0.1,
  );
  elements.guideLabel.textContent = guide.label || "PREPARING";
  elements.guideInstruction.textContent = guide.instruction || "SENSORS SYNCHRONIZING";
  elements.guideDetail.textContent = guide.detail || "Keep the aircraft disarmed and still.";
  elements.guideValue.textContent = readout.value;
  elements.guideEstimate.textContent = readout.estimate;
  elements.guideProgress.style.width = `${readout.progress * 100}%`;
  displayedGuideRevision = revision;
}

function finishGuideFlash() {
  if (!guideFlashActive) return;
  guideFlashActive = false;
  clearTimeout(guideFlashTimeout);
  guideFlashTimeout = null;
  elements.instructionFlash.classList.remove("is-flashing");
  if (pendingGuide !== null) {
    const nextGuide = pendingGuide;
    pendingGuide = null;
    commitGuide(nextGuide.guide, nextGuide.revision);
  }
}

function startGuideFlash() {
  if (guideFlashActive || pendingGuide === null) return;
  guideFlashActive = true;
  elements.instructionFlash.classList.remove("is-flashing");
  void elements.instructionFlash.offsetWidth;
  elements.instructionFlash.classList.add("is-flashing");
  guideFlashTimeout = window.setTimeout(finishGuideFlash, 1200);
}

function renderGuide(guide, session) {
  if (!guide || typeof guide !== "object") return;
  const sequence = Number(guide.sequence) || 0;
  const revision = `${session || "session"}:${sequence}`;
  if (displayedGuideRevision === null && sequence === 0) {
    commitGuide(guide, revision);
    return;
  }
  if (revision === displayedGuideRevision && !guideFlashActive) {
    commitGuide(guide, revision);
    return;
  }
  pendingGuide = { guide, revision };
  startGuideFlash();
}

elements.instructionFlash.addEventListener("animationend", finishGuideFlash);

function renderState(state) {
  latestState = state;
  const rgbd = state.rgbd || {};
  const lio = state.lio || {};
  const imu = state.imu || {};
  const lidar = state.lidar || {};
  const rtlShadow = state.rtl_shadow || {};
  const proofLive = rgbd.connected && lio.publishing && imu.connected && lidar.connected;
  let proofStatus = "PIPELINE SYNCHRONIZING";
  if (state.guide?.complete) proofStatus = "MOTION SEQUENCE COMPLETE";
  else if (state.ready_for_motion) proofStatus = "READY FOR DISARMED MOTION";
  else if (proofLive) proofStatus = "LIVE SHADOW PROOF";
  elements.proofStatus.textContent = proofStatus;
  elements.proofStatus.classList.toggle("is-live", proofLive || state.ready_for_motion);
  elements.sessionName.textContent = state.session || "SESSION --";
  elements.elapsedTime.textContent = clock(state.elapsed_s);
  elements.connectionState.textContent = connected ? "DASHBOARD CONNECTED" : "RECONNECTING";
  elements.mapPoints.textContent = Number(rgbd.map_points || 0).toLocaleString();
  elements.keyframes.textContent = Number(rgbd.map_keyframes || 0).toLocaleString();
  elements.trackRatio.textContent = percent(rgbd.tracking_success_ratio);
  elements.priorRatio.textContent = percent(rgbd.gyro_prior_coverage_ratio);
  elements.rgbdDetail.textContent = `${number(rgbd.measured_fps, 1, " Hz")} / ${rgbd.tracking ? "TRACKING" : "ACQUIRING"}`;
  elements.imuDetail.textContent = `${number(imu.rate_hz, 1, " Hz")} / ${imu.clock_ready ? "SYNCED" : "LOCKING"}`;
  elements.lidarDetail.textContent = `${number(lidar.rate_hz, 1, " Hz")} / ${lidar.clock_ready ? "SYNCED" : "LOCKING"}`;
  elements.lioDetail.textContent = `${Number(lio.rows || 0).toLocaleString()} POSES / ${lio.publishing ? "LIVE" : "WAITING"}`;
  elements.rgbdPath.textContent = number(rgbd.path_length_m, 2, " m");
  elements.lioPath.textContent = number(lio.path_length_m, 2, " m");
  elements.computeTime.textContent = number(rgbd.compute_ms, 1, " ms");
  elements.depthValid.textContent = percent(rgbd.valid_depth_fraction);
  renderRtlShadow(rtlShadow);
  health("rgbd", rgbd.connected && rgbd.tracking, rgbd.connected);
  health("imu", imu.connected && imu.clock_ready && imu.queue_drops === 0, imu.connected);
  health("lidar", lidar.connected && lidar.clock_ready && lidar.queue_drops === 0, lidar.connected);
  health("lio", lio.publishing && lio.synchronized, lio.rows > 0);
  renderGuide(state.guide, state.session);
  updatePath(rgbdLine, state.rgbd_path);
  updatePath(lioLine, lio.path);
  if (Array.isArray(rgbd.position_local_flu_m)) {
    drone.position.copy(scenePoint(rgbd.position_local_flu_m));
  }
}

async function fetchState() {
  try {
    const response = await fetch("/api/slam-poc", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    connected = true;
    renderState(state);
    if (Number(state.map_sequence) !== mapSequence) {
      const mapResponse = await fetch("/api/slam-poc/map", { cache: "no-store" });
      if (mapResponse.ok) {
        const payload = await mapResponse.json();
        updateMap(payload);
        mapSequence = Number(payload.sequence);
      }
    }
  } catch {
    connected = false;
    elements.connectionState.textContent = "RECONNECTING";
  }
}

elements.resetView.addEventListener("click", resetCamera);
elements.topView.addEventListener("click", setTopView);
elements.followView.addEventListener("click", () => {
  following = !following;
  elements.followView.classList.toggle("is-active", following);
});
elements.stopProof.addEventListener("click", async () => {
  elements.stopProof.disabled = true;
  try {
    await fetch("/api/slam-poc/stop", { method: "POST" });
    elements.stopProof.querySelector("span").textContent = "Saving proof";
  } catch {
    elements.stopProof.disabled = false;
  }
});

function resize() {
  const width = Math.max(1, elements.canvas.clientWidth);
  const height = Math.max(1, elements.canvas.clientHeight);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  renderer.setSize(width, height, false);
}

function animate() {
  resize();
  if (following && latestState && !elements.topView.classList.contains("is-active")) {
    const delta = drone.position.clone().sub(controls.target);
    controls.target.lerp(drone.position, 0.04);
    camera.position.addScaledVector(delta, 0.04);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

resetCamera();
fetchState();
setInterval(fetchState, 300);
animate();

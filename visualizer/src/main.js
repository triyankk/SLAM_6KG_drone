import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import {
  Box,
  Circle,
  createIcons,
  Download,
  Pause,
  Play,
  RotateCcw,
  Scan,
  Square,
} from "lucide";
import "./style.css";

const icons = {
  Box,
  Circle,
  Download,
  Pause,
  Play,
  RotateCcw,
  Scan,
  Square,
};

const canvas = document.querySelector("#scene");
const traceCanvas = document.querySelector("#traceCanvas");
const traceContext = traceCanvas.getContext("2d");

const elements = {
  sourceBadge: document.querySelector("#sourceBadge"),
  modeBadge: document.querySelector("#modeBadge"),
  armedBadge: document.querySelector("#armedBadge"),
  linkIndicator: document.querySelector("#linkIndicator"),
  linkText: document.querySelector("#linkText"),
  linkDetail: document.querySelector("#linkDetail"),
  flowX: document.querySelector("#flowX"),
  flowY: document.querySelector("#flowY"),
  flowMagnitude: document.querySelector("#flowMagnitude"),
  flowAge: document.querySelector("#flowAge"),
  qualityValue: document.querySelector("#qualityValue"),
  qualityFill: document.querySelector("#qualityFill"),
  rangeValue: document.querySelector("#rangeValue"),
  rangeLimits: document.querySelector("#rangeLimits"),
  rangeAge: document.querySelector("#rangeAge"),
  rollValue: document.querySelector("#rollValue"),
  pitchValue: document.querySelector("#pitchValue"),
  yawValue: document.querySelector("#yawValue"),
  attitudeAge: document.querySelector("#attitudeAge"),
  accelX: document.querySelector("#accelX"),
  accelY: document.querySelector("#accelY"),
  accelZ: document.querySelector("#accelZ"),
  accelMagnitude: document.querySelector("#accelMagnitude"),
  gyroX: document.querySelector("#gyroX"),
  gyroY: document.querySelector("#gyroY"),
  gyroZ: document.querySelector("#gyroZ"),
  gyroMagnitude: document.querySelector("#gyroMagnitude"),
  imuAge: document.querySelector("#imuAge"),
  imuSource: document.querySelector("#imuSource"),
  modeValue: document.querySelector("#modeValue"),
  systemValue: document.querySelector("#systemValue"),
  pauseButton: document.querySelector("#pauseButton"),
  resetButton: document.querySelector("#resetButton"),
  perspectiveButton: document.querySelector("#perspectiveButton"),
  topButton: document.querySelector("#topButton"),
  vectorScale: document.querySelector("#vectorScale"),
  vectorScaleValue: document.querySelector("#vectorScaleValue"),
  recordButton: document.querySelector("#recordButton"),
  downloadButton: document.querySelector("#downloadButton"),
  displayState: document.querySelector("#displayState"),
};

createIcons({ icons });

function displayPixelRatio() {
  const cap = window.matchMedia("(max-width: 820px)").matches ? 1.25 : 1.5;
  return Math.min(window.devicePixelRatio, cap);
}

const renderer = new THREE.WebGLRenderer({
  canvas,
  antialias: true,
  alpha: false,
  preserveDrawingBuffer: true,
  powerPreference: "high-performance",
});
renderer.setPixelRatio(displayPixelRatio());
renderer.setSize(window.innerWidth, window.innerHeight, false);
renderer.setClearColor(0x111312, 1);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x111312, 0.048);

const camera = new THREE.PerspectiveCamera(
  43,
  window.innerWidth / window.innerHeight,
  0.02,
  80,
);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.065;
controls.minDistance = 1.2;
controls.maxDistance = 16;
controls.maxPolarAngle = Math.PI * 0.49;

scene.add(new THREE.HemisphereLight(0xf0f4f1, 0x20231f, 2.1));
const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
keyLight.position.set(4.5, 7, 3.2);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(512, 512);
keyLight.shadow.camera.left = -5;
keyLight.shadow.camera.right = 5;
keyLight.shadow.camera.top = 5;
keyLight.shadow.camera.bottom = -5;
scene.add(keyLight);

const groundMaterial = new THREE.MeshStandardMaterial({
  color: 0x171a18,
  roughness: 0.98,
  metalness: 0,
});
const ground = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), groundMaterial);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);

const grid = new THREE.GridHelper(20, 40, 0x515853, 0x2d322f);
grid.position.y = 0.003;
grid.material.opacity = 0.56;
grid.material.transparent = true;
scene.add(grid);

function box(width, height, depth, material) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(width, height, depth),
    material,
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function buildDrone() {
  const group = new THREE.Group();
  group.name = "vehicle";

  const frameMaterial = new THREE.MeshStandardMaterial({
    color: 0x3c423f,
    roughness: 0.38,
    metalness: 0.62,
  });
  const shellMaterial = new THREE.MeshStandardMaterial({
    color: 0xdde2df,
    roughness: 0.28,
    metalness: 0.42,
  });
  const motorMaterial = new THREE.MeshStandardMaterial({
    color: 0x171918,
    roughness: 0.42,
    metalness: 0.7,
  });
  const rotorMaterial = new THREE.MeshStandardMaterial({
    color: 0x8e9691,
    transparent: true,
    opacity: 0.44,
    side: THREE.DoubleSide,
    roughness: 0.35,
    metalness: 0.55,
  });

  const body = box(0.38, 0.13, 0.3, shellMaterial);
  body.position.y = 0.015;
  group.add(body);

  const cap = box(0.2, 0.08, 0.18, frameMaterial);
  cap.position.set(-0.02, 0.105, 0);
  group.add(cap);

  const armLength = 0.72;
  const armA = box(armLength, 0.045, 0.055, frameMaterial);
  armA.rotation.y = Math.PI / 4;
  const armB = armA.clone();
  armB.rotation.y = -Math.PI / 4;
  group.add(armA, armB);

  const motorPositions = [
    [0.255, 0.015, 0.255],
    [0.255, 0.015, -0.255],
    [-0.255, 0.015, 0.255],
    [-0.255, 0.015, -0.255],
  ];
  const motorGeometry = new THREE.CylinderGeometry(0.055, 0.062, 0.085, 24);
  const rotorGeometry = new THREE.RingGeometry(0.055, 0.205, 48);
  for (const [x, y, z] of motorPositions) {
    const motor = new THREE.Mesh(motorGeometry, motorMaterial);
    motor.position.set(x, y + 0.04, z);
    motor.castShadow = true;
    group.add(motor);

    const rotor = new THREE.Mesh(rotorGeometry, rotorMaterial);
    rotor.rotation.x = -Math.PI / 2;
    rotor.position.set(x, y + 0.09, z);
    rotor.castShadow = true;
    group.add(rotor);
  }

  const frontMarker = box(
    0.11,
    0.035,
    0.18,
    new THREE.MeshStandardMaterial({
      color: 0xf0b44b,
      emissive: 0x5b3a09,
      emissiveIntensity: 0.5,
    }),
  );
  frontMarker.position.set(0.22, 0.04, 0);
  group.add(frontMarker);

  const sensor = box(
    0.12,
    0.045,
    0.1,
    new THREE.MeshStandardMaterial({
      color: 0x35d8e6,
      emissive: 0x0b4348,
      emissiveIntensity: 0.75,
      roughness: 0.3,
    }),
  );
  sensor.name = "hflow-sensor";
  sensor.position.set(0, -0.1, 0);
  group.add(sensor);

  const lens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.022, 0.027, 0.022, 24),
    new THREE.MeshStandardMaterial({
      color: 0x0b0d0c,
      roughness: 0.15,
      metalness: 0.45,
    }),
  );
  lens.position.set(0, -0.13, 0);
  group.add(lens);

  return group;
}

const drone = buildDrone();
scene.add(drone);
const flowSensor = drone.getObjectByName("hflow-sensor");

const footprint = new THREE.Mesh(
  new THREE.RingGeometry(0.27, 0.285, 64),
  new THREE.MeshBasicMaterial({
    color: 0x35d8e6,
    transparent: true,
    opacity: 0.42,
    side: THREE.DoubleSide,
  }),
);
footprint.rotation.x = -Math.PI / 2;
footprint.position.y = 0.006;
scene.add(footprint);

const rangeLineMaterial = new THREE.LineBasicMaterial({
  color: 0x8de06f,
  transparent: true,
  opacity: 0.9,
});
const rangeLineGeometry = new THREE.BufferGeometry().setFromPoints([
  new THREE.Vector3(),
  new THREE.Vector3(),
]);
const rangeLine = new THREE.Line(rangeLineGeometry, rangeLineMaterial);
scene.add(rangeLine);

const flowArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0.025, 0),
  0.01,
  0x35d8e6,
  0.16,
  0.085,
);
flowArrow.line.material.transparent = true;
flowArrow.line.material.opacity = 0.94;
flowArrow.cone.material.transparent = true;
flowArrow.cone.material.opacity = 0.94;
scene.add(flowArrow);

const accelArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(),
  0.8,
  0x8de06f,
  0.14,
  0.075,
);
const gyroArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(),
  0.2,
  0x70a5ff,
  0.1,
  0.055,
);
scene.add(accelArrow, gyroArrow);

const forwardArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0.018, 0),
  0.62,
  0xf0b44b,
  0.12,
  0.06,
);
const rightArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, 1),
  new THREE.Vector3(0, 0.018, 0),
  0.46,
  0xd8dedb,
  0.1,
  0.055,
);
scene.add(forwardArrow, rightArrow);

const yawQuaternion = new THREE.Quaternion();
const pitchQuaternion = new THREE.Quaternion();
const rollQuaternion = new THREE.Quaternion();
const attitudeQuaternion = new THREE.Quaternion();
const worldFlow = new THREE.Vector3();
const sensorWorld = new THREE.Vector3();
const forwardWorld = new THREE.Vector3();
const rightWorld = new THREE.Vector3();
const accelWorld = new THREE.Vector3();
const gyroWorld = new THREE.Vector3();
const bodyForward = new THREE.Vector3(1, 0, 0);
const bodyRight = new THREE.Vector3(0, 0, 1);
const rangePositions = rangeLineGeometry.attributes.position.array;

let latestTelemetry = null;
let displayTelemetry = null;
let paused = false;
let vectorScale = Number(elements.vectorScale.value);
let recording = false;
let recordedRows = [];
let lastRecordedSequence = -1;
let currentView = "perspective";
let lastUiSequence = -1;
let lastFrameTime = performance.now();
let traceDirty = true;
const flowHistory = [];
const rootStyle = getComputedStyle(document.documentElement);
const traceColors = {
  line: rootStyle.getPropertyValue("--line").trim(),
  x: rootStyle.getPropertyValue("--cyan").trim(),
  y: rootStyle.getPropertyValue("--amber").trim(),
};

function resetPerspective() {
  currentView = "perspective";
  camera.up.set(0, 1, 0);
  camera.position.set(3.4, 2.55, 3.85);
  controls.target.set(0, 0.58, 0);
  controls.enableRotate = true;
  controls.update();
  elements.perspectiveButton.classList.add("is-active");
  elements.topButton.classList.remove("is-active");
}

function setTopView() {
  currentView = "top";
  camera.up.set(0, 0, -1);
  camera.position.set(0.001, 7.2, 0.001);
  controls.target.set(0, 0, 0);
  controls.enableRotate = false;
  controls.update();
  elements.topButton.classList.add("is-active");
  elements.perspectiveButton.classList.remove("is-active");
}

resetPerspective();

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

function formatAge(value) {
  return value === null || value === undefined ? "-- ms" : `${value} ms`;
}

function radiansToDegrees(value) {
  return (finite(value) * 180) / Math.PI;
}

function qualityColor(quality) {
  if (quality >= 150) return "#8de06f";
  if (quality >= 70) return "#f0b44b";
  return "#ef6258";
}

function updateTelemetryUi(data) {
  const flow = data.flow;
  const range = data.range;
  const attitude = data.attitude;
  const imu = data.imu ?? {};
  const vehicle = data.vehicle;
  const linkAge = data.link.age_ms;
  const streamFresh =
    data.link.connected && linkAge !== null && linkAge < 1200;

  elements.sourceBadge.textContent =
    data.source === "demo" ? "DEMO" : "UART";
  const mode = vehicle.mode || "UNKNOWN";
  const forbiddenMode = ["STABILIZE", "GUIDED_NOGPS"].includes(mode);
  elements.modeBadge.textContent = mode;
  elements.modeBadge.classList.toggle("is-forbidden", forbiddenMode);
  elements.armedBadge.textContent = vehicle.armed ? "ARMED" : "DISARMED";
  elements.armedBadge.classList.toggle("is-armed", vehicle.armed);

  elements.linkIndicator.classList.toggle("is-live", streamFresh);
  elements.linkIndicator.classList.toggle(
    "is-stale",
    data.link.connected && !streamFresh,
  );
  elements.linkText.textContent = streamFresh
    ? "Live"
    : data.link.connected
      ? "Stale"
      : "Offline";
  elements.linkDetail.textContent = data.link.detail;

  const flowX = finite(flow.rate_x_rads);
  const flowY = finite(flow.rate_y_rads);
  const magnitude = Math.hypot(flowX, flowY);
  elements.flowX.textContent = flowX.toFixed(3);
  elements.flowY.textContent = flowY.toFixed(3);
  elements.flowMagnitude.textContent = `${magnitude.toFixed(3)} rad/s`;
  elements.flowAge.textContent = formatAge(flow.age_ms);

  const quality = Math.max(0, Math.min(255, finite(flow.quality)));
  elements.qualityValue.textContent = `${Math.round(quality)} / 255`;
  elements.qualityFill.style.width = `${(quality / 255) * 100}%`;
  elements.qualityFill.style.backgroundColor = qualityColor(quality);

  elements.rangeValue.textContent = finite(range.distance_m).toFixed(2);
  elements.rangeLimits.textContent =
    `${finite(range.min_m).toFixed(2)} - ${finite(range.max_m).toFixed(1)} m`;
  elements.rangeAge.textContent = formatAge(range.age_ms);

  elements.rollValue.textContent =
    `${radiansToDegrees(attitude.roll_rad).toFixed(1)} deg`;
  elements.pitchValue.textContent =
    `${radiansToDegrees(attitude.pitch_rad).toFixed(1)} deg`;
  elements.yawValue.textContent =
    `${radiansToDegrees(attitude.yaw_rad).toFixed(1)} deg`;
  elements.attitudeAge.textContent = formatAge(attitude.age_ms);

  const accelX = finite(imu.accel_x_mss);
  const accelY = finite(imu.accel_y_mss);
  const accelZ = finite(imu.accel_z_mss);
  const gyroX = finite(imu.gyro_x_rads);
  const gyroY = finite(imu.gyro_y_rads);
  const gyroZ = finite(imu.gyro_z_rads);
  elements.accelX.textContent = accelX.toFixed(2);
  elements.accelY.textContent = accelY.toFixed(2);
  elements.accelZ.textContent = accelZ.toFixed(2);
  elements.accelMagnitude.textContent =
    `${Math.hypot(accelX, accelY, accelZ).toFixed(2)} m/s2`;
  elements.gyroX.textContent = gyroX.toFixed(3);
  elements.gyroY.textContent = gyroY.toFixed(3);
  elements.gyroZ.textContent = gyroZ.toFixed(3);
  elements.gyroMagnitude.textContent =
    `${Math.hypot(gyroX, gyroY, gyroZ).toFixed(3)} rad/s`;
  elements.imuAge.textContent = formatAge(imu.age_ms);
  elements.imuSource.textContent =
    imu.message && imu.message !== "WAITING"
      ? `${imu.message} - body frame`
      : "Waiting for Cube IMU";

  elements.modeValue.textContent = mode;
  elements.systemValue.textContent =
    vehicle.system_id === null
      ? "--"
      : `${vehicle.system_id}:${vehicle.component_id}`;
}

function updateScene(data, deltaSeconds) {
  const flow = data.flow;
  const attitude = data.attitude;
  const imu = data.imu ?? {};
  const measuredRange = finite(data.range.distance_m);
  const altitude = THREE.MathUtils.clamp(measuredRange + 0.1, 0.14, 5.5);
  const attitudeBlend =
    1 - Math.exp(-THREE.MathUtils.clamp(deltaSeconds, 0, 0.1) / 0.014);
  const altitudeBlend =
    1 - Math.exp(-THREE.MathUtils.clamp(deltaSeconds, 0, 0.1) / 0.07);

  yawQuaternion.setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    -finite(attitude.yaw_rad),
  );
  pitchQuaternion.setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    finite(attitude.pitch_rad),
  );
  rollQuaternion.setFromAxisAngle(
    new THREE.Vector3(1, 0, 0),
    finite(attitude.roll_rad),
  );
  attitudeQuaternion
    .copy(yawQuaternion)
    .multiply(pitchQuaternion)
    .multiply(rollQuaternion);
  drone.quaternion.slerp(attitudeQuaternion, attitudeBlend);
  drone.position.y += (altitude - drone.position.y) * altitudeBlend;

  drone.updateMatrixWorld();
  flowSensor.getWorldPosition(sensorWorld);
  rangePositions[0] = sensorWorld.x;
  rangePositions[1] = sensorWorld.y;
  rangePositions[2] = sensorWorld.z;
  rangePositions[3] = sensorWorld.x;
  rangePositions[4] = 0.012;
  rangePositions[5] = sensorWorld.z;
  rangeLineGeometry.attributes.position.needsUpdate = true;

  footprint.position.x += (sensorWorld.x - footprint.position.x) * 0.2;
  footprint.position.z += (sensorWorld.z - footprint.position.z) * 0.2;

  const flowX = finite(flow.rate_x_rads);
  const flowY = finite(flow.rate_y_rads);
  const magnitude = Math.hypot(flowX, flowY);
  worldFlow.set(flowX, 0, flowY).applyQuaternion(yawQuaternion);
  if (magnitude > 0.0005) {
    worldFlow.normalize();
    flowArrow.setDirection(worldFlow);
    flowArrow.setLength(
      THREE.MathUtils.clamp(magnitude * vectorScale, 0.08, 2.7),
      0.16,
      0.085,
    );
    flowArrow.visible = true;
  } else {
    flowArrow.visible = false;
  }
  flowArrow.position.set(sensorWorld.x, 0.03, sensorWorld.z);

  forwardWorld.copy(bodyForward).applyQuaternion(yawQuaternion);
  rightWorld.copy(bodyRight).applyQuaternion(yawQuaternion);
  forwardArrow.setDirection(forwardWorld);
  rightArrow.setDirection(rightWorld);
  forwardArrow.position.set(sensorWorld.x, 0.02, sensorWorld.z);
  rightArrow.position.set(sensorWorld.x, 0.02, sensorWorld.z);

  accelWorld
    .set(
      finite(imu.accel_x_mss),
      -finite(imu.accel_z_mss),
      finite(imu.accel_y_mss),
    )
    .applyQuaternion(attitudeQuaternion);
  const accelMagnitude = accelWorld.length();
  if (accelMagnitude > 0.02) {
    accelWorld.normalize();
    accelArrow.setDirection(accelWorld);
    const arrowLength = THREE.MathUtils.clamp(
      accelMagnitude * 0.085,
      0.18,
      1.35,
    );
    accelArrow.setLength(
      arrowLength,
      Math.min(0.14, arrowLength * 0.3),
      Math.min(0.075, arrowLength * 0.16),
    );
    accelArrow.visible = true;
  } else {
    accelArrow.visible = false;
  }
  accelArrow.position.set(
    drone.position.x,
    drone.position.y + 0.12,
    drone.position.z,
  );

  gyroWorld
    .set(
      finite(imu.gyro_x_rads),
      -finite(imu.gyro_z_rads),
      finite(imu.gyro_y_rads),
    )
    .applyQuaternion(attitudeQuaternion);
  const gyroMagnitude = gyroWorld.length();
  if (gyroMagnitude > 0.002) {
    gyroWorld.normalize();
    gyroArrow.setDirection(gyroWorld);
    const arrowLength = THREE.MathUtils.clamp(
      gyroMagnitude * 2.2,
      0.14,
      1.15,
    );
    gyroArrow.setLength(
      arrowLength,
      Math.min(0.1, arrowLength * 0.3),
      Math.min(0.055, arrowLength * 0.16),
    );
    gyroArrow.visible = true;
  } else {
    gyroArrow.visible = false;
  }
  gyroArrow.position.set(
    drone.position.x - 0.12,
    drone.position.y + 0.23,
    drone.position.z,
  );

  if (currentView === "perspective") {
    controls.target.y += (altitude * 0.42 - controls.target.y) * 0.04;
  }
}

function appendHistory(data) {
  const now = performance.now() / 1000;
  flowHistory.push({
    time: now,
    x: finite(data.flow.rate_x_rads),
    y: finite(data.flow.rate_y_rads),
  });
  while (flowHistory.length && now - flowHistory[0].time > 10) {
    flowHistory.shift();
  }
  traceDirty = true;
}

function drawTrace() {
  const width = traceCanvas.width;
  const height = traceCanvas.height;
  traceContext.clearRect(0, 0, width, height);
  traceContext.fillStyle = "rgba(17, 19, 18, 0.34)";
  traceContext.fillRect(0, 0, width, height);

  traceContext.strokeStyle = traceColors.line;
  traceContext.lineWidth = 1;
  for (let index = 1; index < 4; index += 1) {
    const y = (height * index) / 4;
    traceContext.beginPath();
    traceContext.moveTo(0, y);
    traceContext.lineTo(width, y);
    traceContext.stroke();
  }

  if (flowHistory.length < 2) {
    traceDirty = false;
    return;
  }
  const end = flowHistory[flowHistory.length - 1].time;
  const values = flowHistory.flatMap((sample) => [sample.x, sample.y]);
  const limit = Math.max(0.15, ...values.map((value) => Math.abs(value)));

  function plot(key, color) {
    traceContext.strokeStyle = color;
    traceContext.lineWidth = 3;
    traceContext.beginPath();
    flowHistory.forEach((sample, index) => {
      const x = width - ((end - sample.time) / 10) * width;
      const y = height / 2 - (sample[key] / limit) * (height * 0.4);
      if (index === 0) traceContext.moveTo(x, y);
      else traceContext.lineTo(x, y);
    });
    traceContext.stroke();
  }

  plot("x", traceColors.x);
  plot("y", traceColors.y);
  traceDirty = false;
}

function captureRecording(data) {
  if (!recording || data.sequence === lastRecordedSequence) return;
  lastRecordedSequence = data.sequence;
  recordedRows.push({
    client_time_iso: new Date().toISOString(),
    source: data.source,
    flow_x_rads: finite(data.flow.rate_x_rads),
    flow_y_rads: finite(data.flow.rate_y_rads),
    quality: finite(data.flow.quality),
    distance_m: finite(data.range.distance_m),
    roll_rad: finite(data.attitude.roll_rad),
    pitch_rad: finite(data.attitude.pitch_rad),
    yaw_rad: finite(data.attitude.yaw_rad),
    accel_x_mss: finite(data.imu?.accel_x_mss),
    accel_y_mss: finite(data.imu?.accel_y_mss),
    accel_z_mss: finite(data.imu?.accel_z_mss),
    gyro_x_rads: finite(data.imu?.gyro_x_rads),
    gyro_y_rads: finite(data.imu?.gyro_y_rads),
    gyro_z_rads: finite(data.imu?.gyro_z_rads),
    flow_age_ms: data.flow.age_ms ?? "",
    imu_age_ms: data.imu?.age_ms ?? "",
  });
  elements.downloadButton.disabled = recordedRows.length === 0;
}

function replaceButtonIcon(button, iconName) {
  button.innerHTML = `<i data-lucide="${iconName}"></i>`;
  createIcons({ icons });
}

function downloadCsv() {
  if (!recordedRows.length) return;
  const headers = Object.keys(recordedRows[0]);
  const lines = [
    headers.join(","),
    ...recordedRows.map((row) =>
      headers
        .map((header) => JSON.stringify(row[header] ?? ""))
        .join(","),
    ),
  ];
  const blob = new Blob([`${lines.join("\n")}\n`], {
    type: "text/csv;charset=utf-8",
  });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `hflow-${new Date().toISOString().replaceAll(":", "-")}.csv`;
  link.click();
  URL.revokeObjectURL(link.href);
}

elements.pauseButton.addEventListener("click", () => {
  paused = !paused;
  elements.displayState.hidden = !paused;
  elements.pauseButton.title = paused ? "Resume display" : "Pause display";
  elements.pauseButton.setAttribute(
    "aria-label",
    paused ? "Resume display" : "Pause display",
  );
  replaceButtonIcon(elements.pauseButton, paused ? "play" : "pause");
});

elements.resetButton.addEventListener("click", resetPerspective);
elements.perspectiveButton.addEventListener("click", resetPerspective);
elements.topButton.addEventListener("click", setTopView);
elements.vectorScale.addEventListener("input", () => {
  vectorScale = Number(elements.vectorScale.value);
  elements.vectorScaleValue.textContent = `${vectorScale.toFixed(1)}x`;
});
elements.recordButton.addEventListener("click", () => {
  recording = !recording;
  elements.recordButton.classList.toggle("is-recording", recording);
  elements.recordButton.title = recording
    ? "Stop recording"
    : "Record telemetry";
  elements.recordButton.setAttribute(
    "aria-label",
    recording ? "Stop recording" : "Record telemetry",
  );
  replaceButtonIcon(elements.recordButton, recording ? "square" : "circle");
});
elements.downloadButton.addEventListener("click", downloadCsv);

const eventSource = new EventSource("/api/stream");
eventSource.onmessage = (event) => {
  try {
    latestTelemetry = JSON.parse(event.data);
    if (!paused) {
      displayTelemetry = latestTelemetry;
      appendHistory(displayTelemetry);
    }
    captureRecording(latestTelemetry);
  } catch (error) {
    console.error("Invalid telemetry payload", error);
  }
};
eventSource.onerror = () => {
  elements.linkIndicator.classList.remove("is-live");
  elements.linkIndicator.classList.add("is-stale");
  elements.linkText.textContent = "Reconnecting";
};

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setPixelRatio(displayPixelRatio());
  renderer.setSize(width, height, false);
}

window.addEventListener("resize", resize);

function animate(frameTime) {
  requestAnimationFrame(animate);
  const deltaSeconds = Math.min((frameTime - lastFrameTime) / 1000, 0.1);
  lastFrameTime = frameTime;
  if (displayTelemetry) {
    if (displayTelemetry.sequence !== lastUiSequence) {
      updateTelemetryUi(displayTelemetry);
      lastUiSequence = displayTelemetry.sequence;
    }
    updateScene(displayTelemetry, deltaSeconds);
  }
  if (traceDirty) drawTrace();
  controls.update();
  renderer.render(scene, camera);
}

requestAnimationFrame(animate);

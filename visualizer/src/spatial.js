import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { buildDrone } from "./drone.js";

const MAX_ROLLING_POINTS = 80_000;
const MAX_POSE_HISTORY = 300;
const MAX_TRAIL_POINTS = 900;
const MIN_DISPLAY_TRAJECTORY_STEP_M = 0.06;
const MAX_DISPLAY_TRAJECTORY_STEP_M = 0.35;

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

function formatAge(value) {
  return value === null || value === undefined ? "-- ms" : `${value} ms`;
}

function decodeBase64(value) {
  const binary = atob(value);
  const output = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    output[index] = binary.charCodeAt(index);
  }
  return output;
}

function composeSceneAttitude(target, attitude) {
  const yaw = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    -finite(attitude?.yaw_rad),
  );
  const pitch = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    finite(attitude?.pitch_rad),
  );
  const roll = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(1, 0, 0),
    finite(attitude?.roll_rad),
  );
  return target.copy(yaw).multiply(pitch).multiply(roll);
}

function poseFromTelemetry(data, origin = null) {
  const local = data?.local_position ?? {};
  const rawX = finite(local.x_m);
  const rawY = finite(local.y_m);
  const rawZDown = finite(local.z_down_m);
  return {
    time: finite(data?.server_monotonic_s, performance.now() / 1000),
    fresh:
      local.age_ms !== null &&
      local.age_ms !== undefined &&
      local.age_ms < 1200,
    position: new THREE.Vector3(
      origin === null ? rawX : rawX - origin.x,
      origin === null
        ? -rawZDown
        : -(rawZDown - origin.zDown) + origin.initialUp,
      origin === null ? rawY : rawY - origin.y,
    ),
    quaternion: composeSceneAttitude(
      new THREE.Quaternion(),
      data?.attitude ?? {},
    ),
  };
}

function sourceLabel(source) {
  return source === "depth_camera" ? "D415" : "JT16";
}

export function createSpatialVisualizer() {
  const canvas = document.querySelector("#mapScene");
  const elements = {
    connection: document.querySelector("#spatialConnection"),
    frameBadge: document.querySelector("#spatialFrameBadge"),
    mapPoints: document.querySelector("#mapPoints"),
    mapFrames: document.querySelector("#mapFrames"),
    droppedFrames: document.querySelector("#droppedFrames"),
    clearancePanel: document.querySelector("#clearancePanel"),
    clearanceStatus: document.querySelector("#clearanceStatus"),
    clearanceLimit: document.querySelector("#clearanceLimit"),
    clearanceNearest: document.querySelector("#clearanceNearest"),
    clearanceMargin: document.querySelector("#clearanceMargin"),
    poseStatus: document.querySelector("#poseStatus"),
    poseX: document.querySelector("#poseX"),
    poseY: document.querySelector("#poseY"),
    poseZ: document.querySelector("#poseZ"),
    poseSpeed: document.querySelector("#poseSpeed"),
    depthIndicator: document.querySelector("#depthCloudIndicator"),
    depthStatus: document.querySelector("#depthCloudStatus"),
    depthRate: document.querySelector("#depthCloudRate"),
    depthPoints: document.querySelector("#depthCloudPoints"),
    depthAge: document.querySelector("#depthCloudAge"),
    lidarIndicator: document.querySelector("#lidarCloudIndicator"),
    lidarStatus: document.querySelector("#lidarCloudStatus"),
    lidarRate: document.querySelector("#lidarCloudRate"),
    lidarPoints: document.querySelector("#lidarCloudPoints"),
    lidarAge: document.querySelector("#lidarCloudAge"),
    depthVisibility: document.querySelector("#depthVisibility"),
    lidarVisibility: document.querySelector("#lidarVisibility"),
    resetButton: document.querySelector("#mapResetButton"),
    topButton: document.querySelector("#mapTopButton"),
    followButton: document.querySelector("#mapFollowButton"),
    trailButton: document.querySelector("#mapTrailButton"),
    trajectoryButton: document.querySelector("#mapTrajectoryButton"),
    clearButton: document.querySelector("#mapClearButton"),
    slamStatusBadge: document.querySelector("#slamStatusBadge"),
    worldPoseContract: document.querySelector("#worldPoseContract"),
    historyContract: document.querySelector("#historyContract"),
    trajectoryDetail: document.querySelector("#trajectoryDetail"),
    spatialView: document.querySelector("#spatialView"),
    pointSize: document.querySelector("#pointSize"),
    pointSizeValue: document.querySelector("#pointSizeValue"),
    trailSeconds: document.querySelector("#trailSeconds"),
    trailSecondsValue: document.querySelector("#trailSecondsValue"),
  };

  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: false,
    preserveDrawingBuffer: true,
    powerPreference: "high-performance",
  });
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.setClearColor(0x0c0f10, 1);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x0c0f10, 0.018);
  const camera = new THREE.PerspectiveCamera(48, 1, 0.025, 120);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.minDistance = 1.0;
  controls.maxDistance = 45;
  controls.maxPolarAngle = Math.PI * 0.495;

  scene.add(new THREE.HemisphereLight(0xdde8e5, 0x15191a, 1.8));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2.3);
  keyLight.position.set(8, 12, 7);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(1024, 1024);
  scene.add(keyLight);

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(60, 60),
    new THREE.MeshStandardMaterial({
      color: 0x121617,
      roughness: 0.96,
      metalness: 0.04,
    }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.015;
  floor.receiveShadow = true;
  scene.add(floor);

  const grid = new THREE.GridHelper(60, 120, 0x4b5757, 0x252d2e);
  grid.material.transparent = true;
  grid.material.opacity = 0.56;
  scene.add(grid);

  const mapDrone = buildDrone();
  scene.add(mapDrone);
  const mapCube = mapDrone.getObjectByName("cube-orange");

  const airframeRing = new THREE.Mesh(
    new THREE.RingGeometry(0.6536, 0.75, 96),
    new THREE.MeshBasicMaterial({
      color: 0xf0b44b,
      transparent: true,
      opacity: 0.17,
      side: THREE.DoubleSide,
      depthWrite: false,
    }),
  );
  airframeRing.rotation.x = -Math.PI / 2;
  scene.add(airframeRing);

  const clearanceZoneMaterial = new THREE.MeshBasicMaterial({
    color: 0xf0b44b,
    transparent: true,
    opacity: 0.045,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  const clearanceZone = new THREE.Mesh(
    new THREE.CircleGeometry(2.0, 128),
    clearanceZoneMaterial,
  );
  clearanceZone.rotation.x = -Math.PI / 2;
  scene.add(clearanceZone);

  const clearanceBoundaryMaterial = new THREE.MeshBasicMaterial({
    color: 0xf0b44b,
    transparent: true,
    opacity: 0.82,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  const clearanceBoundary = new THREE.Mesh(
    new THREE.RingGeometry(1.97, 2.0, 128),
    clearanceBoundaryMaterial,
  );
  clearanceBoundary.rotation.x = -Math.PI / 2;
  scene.add(clearanceBoundary);

  const originAxes = new THREE.AxesHelper(1.2);
  originAxes.position.y = 0.02;
  scene.add(originAxes);

  let trailGeometry = new THREE.BufferGeometry();
  const trailMaterial = new THREE.LineBasicMaterial({
    color: 0xf0b44b,
    transparent: true,
    opacity: 0.82,
    depthTest: false,
    depthWrite: false,
  });
  const trajectory = new THREE.Line(trailGeometry, trailMaterial);
  trajectory.frustumCulled = false;
  trajectory.renderOrder = 29;
  scene.add(trajectory);

  function updateTelemetryTrail(points) {
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    trailGeometry.dispose();
    trailGeometry = geometry;
    trajectory.geometry = geometry;
  }

  function makeNavigationLine(color, opacity = 0.9, dashed = false) {
    const geometry = new LineGeometry();
    const material = new LineMaterial({
      color,
      linewidth: dashed ? 2.6 : 3.8,
      dashed,
      dashSize: 0.11,
      gapSize: 0.07,
      transparent: true,
      opacity,
      depthTest: false,
      depthWrite: false,
      alphaToCoverage: true,
      fog: false,
    });
    const line = new Line2(geometry, material);
    line.frustumCulled = false;
    line.renderOrder = 30;
    line.visible = false;
    return line;
  }

  function makeNavigationMarker(color, radius = 0.065) {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(radius, 18, 12),
      new THREE.MeshBasicMaterial({
        color,
        depthTest: false,
        depthWrite: false,
      }),
    );
    marker.renderOrder = 31;
    marker.visible = false;
    return marker;
  }

  const navigationTrajectories = new THREE.Group();
  const navigationLines = {
    lio: makeNavigationLine(0x35d8e6, 0.98),
    rgbd: makeNavigationLine(0x8de06f, 0.88),
    cube: makeNavigationLine(0xe8eeeb, 0.72),
    breadcrumbs: makeNavigationLine(0xf0b44b, 0.98, true),
  };
  for (const line of Object.values(navigationLines)) {
    navigationTrajectories.add(line);
  }
  const navigationMarkers = {
    lio: makeNavigationMarker(0x35d8e6, 0.075),
    rgbd: makeNavigationMarker(0x8de06f, 0.064),
    cube: makeNavigationMarker(0xe8eeeb, 0.052),
    breadcrumbs: makeNavigationMarker(0xf0b44b, 0.058),
  };
  for (const marker of Object.values(navigationMarkers)) {
    navigationTrajectories.add(marker);
  }
  const returnTarget = new THREE.Mesh(
    new THREE.SphereGeometry(0.095, 24, 16),
    new THREE.MeshBasicMaterial({ color: 0xf0655d }),
  );
  returnTarget.visible = false;
  navigationTrajectories.add(returnTarget);
  scene.add(navigationTrajectories);

  const scanFrames = [];
  const poseHistory = [];
  const trajectoryPoints = [];
  const sourceState = {
    depth_camera: {
      connected: false,
      detail: "Waiting",
      frameRate: 0,
      points: 0,
      lastFrameAt: null,
      visible: true,
    },
    lidar: {
      connected: false,
      detail: "Waiting",
      frameRate: 0,
      points: 0,
      lastFrameAt: null,
      visible: false,
    },
  };

  let active = false;
  let following = true;
  let rolling = true;
  let pointSize = Number(elements.pointSize.value);
  let historySeconds = Number(elements.trailSeconds.value);
  let droppedFrames = 0;
  let lastUiUpdate = 0;
  let lastDronePosition = new THREE.Vector3();
  let hasDronePosition = false;
  let eventSource = null;
  let trajectoryEventSource = null;
  let viewOrigin = null;
  let hardCgClearanceM = 1.5;
  let navigationTrajectoriesVisible = true;
  let trajectoryAlignmentOffset = null;
  let lastTrajectoryPoseSequence = null;

  function displayPixelRatio() {
    const cap = window.matchMedia("(max-width: 820px)").matches ? 1.15 : 1.4;
    return Math.min(window.devicePixelRatio, cap);
  }

  function resize() {
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setPixelRatio(displayPixelRatio());
    renderer.setSize(width, height, false);
    for (const line of Object.values(navigationLines)) {
      line.material.resolution.set(width, height);
    }
  }

  function resetCamera() {
    const center = mapDrone.position;
    camera.up.set(0, 1, 0);
    camera.position.set(center.x + 6.5, center.y + 4.6, center.z + 7.2);
    controls.target.copy(center).add(new THREE.Vector3(0, 0.25, 0));
    controls.enableRotate = true;
    controls.update();
    elements.topButton.classList.remove("is-active");
  }

  function setTopView() {
    const center = mapDrone.position;
    camera.up.set(0, 0, -1);
    camera.position.set(center.x + 0.001, center.y + 14, center.z + 0.001);
    controls.target.copy(center);
    controls.enableRotate = false;
    controls.update();
    elements.topButton.classList.add("is-active");
  }

  function disposeFrame(frame) {
    scene.remove(frame.object);
    frame.object.geometry.dispose();
    frame.object.material.dispose();
  }

  function clearFrames() {
    while (scanFrames.length) disposeFrame(scanFrames.pop());
    trajectoryPoints.length = 0;
    updateTelemetryTrail([]);
  }

  function navigationPoint(values, yawRad) {
    if (!Array.isArray(values) || values.length !== 3) return null;
    const forward = Number(values[0]);
    const left = Number(values[1]);
    const up = Number(values[2]);
    if (![forward, left, up].every(Number.isFinite)) return null;
    const right = -left;
    const cosine = Math.cos(yawRad);
    const sine = Math.sin(yawRad);
    return new THREE.Vector3(
      cosine * forward - sine * right,
      up,
      sine * forward + cosine * right,
    );
  }

  function setNavigationLine(
    line,
    marker,
    values,
    yawRad,
    offset,
    minimumStepM = MIN_DISPLAY_TRAJECTORY_STEP_M,
  ) {
    const rawPoints = [];
    for (const value of Array.isArray(values) ? values : []) {
      const point = navigationPoint(value, yawRad);
      if (point !== null) rawPoints.push(point.add(offset));
    }
    const smoothedPoints = rawPoints.map((point, index) => {
      if (index === 0 || index === rawPoints.length - 1) return point.clone();
      return rawPoints[index - 1]
        .clone()
        .multiplyScalar(0.25)
        .addScaledVector(point, 0.5)
        .addScaledVector(rawPoints[index + 1], 0.25);
    });
    const points = [];
    let rejected = 0;
    for (let index = 0; index < smoothedPoints.length; index += 1) {
      if (
        index > 0 &&
        rawPoints[index].distanceTo(rawPoints[index - 1]) >
          MAX_DISPLAY_TRAJECTORY_STEP_M
      ) {
        rejected = smoothedPoints.length - index;
        break;
      }
      const point = smoothedPoints[index];
      if (
        !points.length ||
        point.distanceTo(points[points.length - 1]) >= minimumStepM
      ) {
        points.push(point);
      }
    }
    const latest = smoothedPoints[smoothedPoints.length - 1];
    if (
      rejected === 0 &&
      latest &&
      points.length &&
      latest.distanceTo(points[points.length - 1]) >= minimumStepM * 0.5
    ) {
      points.push(latest);
    }
    if (points.length >= 2) {
      const geometry = new LineGeometry();
      geometry.setPositions(points.flatMap((point) => point.toArray()));
      line.geometry.dispose();
      line.geometry = geometry;
    }
    line.visible = points.length >= 2;
    marker.visible = points.length >= 1;
    if (points.length) marker.position.copy(points[points.length - 1]);
    if (line.material.dashed && points.length >= 2) {
      line.computeLineDistances();
    }
    return { points, rejected };
  }

  function navigationPathLength(points) {
    let length = 0;
    for (let index = 1; index < points.length; index += 1) {
      length += points[index - 1].distanceTo(points[index]);
    }
    return length;
  }

  function updateTrajectoryBadge(payload, metrics = null) {
    const available = Boolean(payload?.available);
    const live = Boolean(payload?.live);
    const state = String(payload?.state ?? "").replaceAll("_", " ");
    const poseReason = String(payload?.estimator?.pose_reason ?? "unknown");
    const poseReady = poseReason === "ready";
    elements.slamStatusBadge.classList.toggle(
      "is-live",
      available && live && poseReady,
    );
    elements.slamStatusBadge.classList.toggle(
      "is-stale",
      available && (!live || !poseReady),
    );
    let badgeText = "SLAM OFFLINE";
    if (available && !live) badgeText = "SLAM STALE";
    if (available && live && !poseReady) {
      badgeText = poseReason.includes("jump") ? "SLAM FAULT" : "SLAM WAIT";
    }
    if (available && live && poseReady) {
      badgeText = payload?.shadow_only
        ? "SLAM MONITOR"
        : state
          ? state.toUpperCase()
          : "SLAM LIVE";
    }
    elements.slamStatusBadge.textContent = badgeText;
    elements.slamStatusBadge.title = `${payload?.detail ?? state} / ${poseReason}`;
    elements.worldPoseContract.textContent = available
      ? "SLAM LOCAL"
      : "CUBE LOCAL";
    elements.historyContract.textContent = available
      ? "BREADCRUMBS"
      : "ROLLING";
    const age = Number(payload?.age_ms);
    const ageLabel = Number.isFinite(age) ? `${Math.round(age)} ms` : "-- ms";
    const output = payload?.shadow_only ? "MONITOR" : "ACTIVE";
    const pathLabel = metrics
      ? ` / LIO ${metrics.lio.length.toFixed(2)} m / RGB-D ${metrics.rgbd.length.toFixed(2)} m / POSE ${poseReason.replaceAll("_", " ").toUpperCase()}`
      : "";
    elements.trajectoryDetail.textContent = available
      ? `${state.toUpperCase() || "WAITING"} / ${output} / ${ageLabel}${pathLabel}`
      : "SLAM STATUS OFFLINE";
  }

  function ingestNavigationTrajectory(payload) {
    if (!payload?.available) {
      updateTrajectoryBadge(payload);
      navigationTrajectories.visible = false;
      trajectory.visible = true;
      return;
    }
    trajectory.visible = false;
    const paths = payload.trajectories ?? {};
    const lioDisplayPath =
      Array.isArray(paths.lio_monitor) && paths.lio_monitor.length
        ? paths.lio_monitor
        : paths.lio;
    const yawRad = finite(payload?.estimator?.frame_yaw_ned_rad);
    const poseSequence = Number(payload?.estimator?.pose_sequence);
    if (
      Number.isFinite(poseSequence) &&
      lastTrajectoryPoseSequence !== null &&
      poseSequence < lastTrajectoryPoseSequence
    ) {
      trajectoryAlignmentOffset = null;
      viewOrigin = null;
      poseHistory.length = 0;
      trajectoryPoints.length = 0;
      updateTelemetryTrail([]);
    }
    if (Number.isFinite(poseSequence)) {
      lastTrajectoryPoseSequence = poseSequence;
    }
    const alignmentSource =
      Array.isArray(lioDisplayPath) && lioDisplayPath.length
        ? lioDisplayPath[lioDisplayPath.length - 1]
        : Array.isArray(paths.cube) && paths.cube.length
          ? paths.cube[paths.cube.length - 1]
          : null;
    if (alignmentSource !== null) {
      const latest = navigationPoint(alignmentSource, yawRad);
      if (latest !== null) {
        trajectoryAlignmentOffset = mapDrone.position.clone().sub(latest);
      }
    }
    const offset = trajectoryAlignmentOffset ?? new THREE.Vector3();
    const displayedPaths = {
      lio: setNavigationLine(
        navigationLines.lio,
        navigationMarkers.lio,
        lioDisplayPath,
        yawRad,
        offset,
      ),
      rgbd: setNavigationLine(
        navigationLines.rgbd,
        navigationMarkers.rgbd,
        paths.rgbd,
        yawRad,
        offset,
      ),
      cube: setNavigationLine(
        navigationLines.cube,
        navigationMarkers.cube,
        paths.cube,
        yawRad,
        offset,
      ),
    };
    displayedPaths.breadcrumbs = setNavigationLine(
      navigationLines.breadcrumbs,
      navigationMarkers.breadcrumbs,
      paths.breadcrumbs,
      yawRad,
      offset,
      0.02,
    );
    const target = navigationPoint(paths.target, yawRad);
    returnTarget.visible = target !== null;
    if (target !== null) returnTarget.position.copy(target.add(offset));
    navigationTrajectories.visible = navigationTrajectoriesVisible;
    const metrics = Object.fromEntries(
      Object.entries(displayedPaths).map(([name, points]) => [
        name,
        {
          points: points.points.length,
          length: navigationPathLength(points.points),
          rejected: points.rejected,
        },
      ]),
    );
    elements.spatialView.dataset.trajectoryPoints = String(
      metrics.lio.points + metrics.rgbd.points + metrics.cube.points,
    );
    elements.spatialView.dataset.lioPathM = metrics.lio.length.toFixed(3);
    elements.spatialView.dataset.rgbdPathM = metrics.rgbd.length.toFixed(3);
    elements.spatialView.dataset.trajectoryRejected = String(
      metrics.lio.rejected + metrics.rgbd.rejected + metrics.cube.rejected,
    );
    updateTrajectoryBadge(payload, metrics);
  }

  function removeSourceFrames(source) {
    for (let index = scanFrames.length - 1; index >= 0; index -= 1) {
      if (scanFrames[index].source !== source) continue;
      disposeFrame(scanFrames[index]);
      scanFrames.splice(index, 1);
    }
  }

  function closestPose(frameTime) {
    if (!poseHistory.length) {
      return {
        position: mapDrone.position.clone(),
        quaternion: mapDrone.quaternion.clone(),
      };
    }
    let closest = poseHistory[poseHistory.length - 1];
    let difference = Math.abs(closest.time - frameTime);
    for (let index = poseHistory.length - 2; index >= 0; index -= 1) {
      const candidate = poseHistory[index];
      const candidateDifference = Math.abs(candidate.time - frameTime);
      if (candidateDifference >= difference) break;
      closest = candidate;
      difference = candidateDifference;
    }
    return closest;
  }

  function ingestTelemetry(data) {
    if (!data) return;
    const local = data.local_position ?? {};
    if (
      viewOrigin === null &&
      local.age_ms !== null &&
      local.age_ms !== undefined &&
      local.age_ms < 1200
    ) {
      viewOrigin = {
        x: finite(local.x_m),
        y: finite(local.y_m),
        zDown: finite(local.z_down_m),
        initialUp: THREE.MathUtils.clamp(
          finite(data.range?.distance_m, 0.15),
          0.08,
          3.0,
        ),
      };
    }
    const pose = poseFromTelemetry(data, viewOrigin);
    poseHistory.push(pose);
    if (poseHistory.length > MAX_POSE_HISTORY) poseHistory.shift();
  }

  function decodeFrame(event) {
    if (
      event.encoding !== "int16_le_base64" ||
      event.frame_id !== "body_frd"
    ) {
      throw new Error("Unsupported spatial frame encoding");
    }
    const pointBytes = decodeBase64(event.points_b64);
    const colors = decodeBase64(event.colors_b64);
    const count = Number(event.point_count);
    if (pointBytes.byteLength !== count * 6 || colors.byteLength !== count * 3) {
      throw new Error("Spatial frame byte length is invalid");
    }

    const dataView = new DataView(
      pointBytes.buffer,
      pointBytes.byteOffset,
      pointBytes.byteLength,
    );
    const positions = new Float32Array(count * 3);
    const scale = finite(event.scale_m, 0.01);
    for (let index = 0; index < count; index += 1) {
      const offset = index * 6;
      const forward = dataView.getInt16(offset, true) * scale;
      const right = dataView.getInt16(offset + 2, true) * scale;
      const down = dataView.getInt16(offset + 4, true) * scale;
      positions[index * 3] = forward;
      positions[index * 3 + 1] = -down;
      positions[index * 3 + 2] = right;
    }
    return { positions, colors };
  }

  function addFrame(event) {
    const source = event.source;
    if (!(source in sourceState)) return;
    const state = sourceState[source];
    state.connected = true;
    state.detail = `${sourceLabel(source)} live`;
    state.frameRate = finite(event.frame_rate_hz);
    state.points = Number(event.point_count);
    state.lastFrameAt = performance.now();
    droppedFrames += Number(event.dropped_before ?? 0);
    if (!state.visible) return;
    if (!rolling) removeSourceFrames(source);
    const decoded = decodeFrame(event);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(decoded.positions, 3),
    );
    geometry.setAttribute(
      "color",
      new THREE.BufferAttribute(decoded.colors, 3, true),
    );
    geometry.computeBoundingSphere();
    const material = new THREE.PointsMaterial({
      size: pointSize,
      sizeAttenuation: true,
      vertexColors: true,
      transparent: true,
      opacity: source === "depth_camera" ? 0.92 : 0.84,
      depthWrite: false,
    });
    const object = new THREE.Points(geometry, material);
    const frameTime = finite(event.frame_monotonic_ns) / 1e9;
    const pose = closestPose(frameTime);
    object.position.copy(pose.position);
    object.quaternion.copy(pose.quaternion);
    object.visible = sourceState[source].visible;
    scene.add(object);
    scanFrames.push({
      source,
      object,
      points: Number(event.point_count),
      receivedAt: performance.now() / 1000,
    });

  }

  function updateSourceStatus(source, payload) {
    if (!(source in sourceState)) return;
    const state = sourceState[source];
    state.connected = Boolean(payload.connected);
    state.detail = payload.detail ?? state.detail;
    state.frameRate = finite(payload.frame_rate_hz, state.frameRate);
    state.points = finite(payload.display_points, state.points);
  }

  function connectSpatialStream() {
    if (eventSource !== null) return;
    elements.connection.textContent = "STREAM CONNECTING";
    elements.connection.classList.remove("is-live");
    eventSource = new EventSource("/api/spatial");
    eventSource.onopen = () => {
      elements.connection.textContent = "STREAM CONNECTED";
      elements.connection.classList.add("is-live");
    };
    eventSource.onmessage = (message) => {
      try {
        const payload = JSON.parse(message.data);
        if (payload.kind === "snapshot") {
          for (const [source, status] of Object.entries(
            payload.sources ?? {},
          )) {
            updateSourceStatus(source, status);
          }
          return;
        }
        if (payload.kind === "status") {
          updateSourceStatus(payload.source, payload);
          droppedFrames += Number(payload.dropped_before ?? 0);
          return;
        }
        if (payload.kind === "frame") addFrame(payload);
      } catch (error) {
        console.error("Invalid spatial payload", error);
      }
    };
    eventSource.onerror = () => {
      elements.connection.textContent = "STREAM RECONNECTING";
      elements.connection.classList.remove("is-live");
    };
  }

  function disconnectSpatialStream() {
    if (eventSource !== null) eventSource.close();
    eventSource = null;
    elements.connection.textContent = "STREAM PAUSED";
    elements.connection.classList.remove("is-live");
  }

  function connectTrajectoryStream() {
    if (trajectoryEventSource !== null) return;
    trajectoryEventSource = new EventSource("/api/trajectory");
    trajectoryEventSource.onmessage = (message) => {
      try {
        ingestNavigationTrajectory(JSON.parse(message.data));
      } catch (error) {
        console.error("Invalid trajectory payload", error);
      }
    };
    trajectoryEventSource.onerror = () => {
      updateTrajectoryBadge({
        available: false,
        detail: "Trajectory stream reconnecting",
      });
    };
  }

  function disconnectTrajectoryStream() {
    if (trajectoryEventSource !== null) trajectoryEventSource.close();
    trajectoryEventSource = null;
  }

  function pruneFrames(nowSeconds) {
    let pointTotal = scanFrames.reduce(
      (total, frame) => total + frame.points,
      0,
    );
    for (let index = scanFrames.length - 1; index >= 0; index -= 1) {
      const frame = scanFrames[index];
      const age = nowSeconds - frame.receivedAt;
      frame.object.material.opacity = rolling
        ? Math.max(0.1, 1 - age / historySeconds) *
          (frame.source === "depth_camera" ? 0.92 : 0.84)
        : frame.source === "depth_camera"
          ? 0.92
          : 0.84;
    }
    while (
      scanFrames.length &&
      (nowSeconds - scanFrames[0].receivedAt > historySeconds ||
        pointTotal > MAX_ROLLING_POINTS)
    ) {
      const frame = scanFrames.shift();
      pointTotal -= frame.points;
      disposeFrame(frame);
    }
    return pointTotal;
  }

  function updateSourceUi(source, now) {
    const state = sourceState[source];
    const age =
      state.lastFrameAt === null ? null : Math.max(0, Math.round(now - state.lastFrameAt));
    const fresh = state.connected && age !== null && age < 1000;
    const prefix = source === "depth_camera" ? "depth" : "lidar";
    elements[`${prefix}Indicator`].classList.toggle("is-live", fresh);
    elements[`${prefix}Indicator`].classList.toggle(
      "is-stale",
      state.connected && !fresh,
    );
    elements[`${prefix}Status`].textContent = fresh
      ? "LIVE"
      : state.connected
        ? "STALE"
        : "OFFLINE";
    elements[`${prefix}Status`].title = state.detail;
    elements[`${prefix}Rate`].textContent = `${state.frameRate.toFixed(1)} Hz`;
    elements[`${prefix}Points`].textContent = state.points.toLocaleString();
    elements[`${prefix}Age`].textContent = formatAge(age);
  }

  function updateUi(data, pointTotal, now) {
    const local = data?.local_position ?? {};
    const speed = Math.hypot(
      finite(local.vx_mps),
      finite(local.vy_mps),
      finite(local.vz_mps),
    );
    const poseFresh =
      local.age_ms !== null &&
      local.age_ms !== undefined &&
      local.age_ms < 1200;
    elements.poseStatus.textContent = poseFresh
      ? data?.source === "slam_runtime"
        ? local.source === "lio_monitor"
          ? "LIO RAW MONITOR"
          : "SLAM LOCAL LIVE"
        : "CUBE LOCAL LIVE"
      : "BODY ORIGIN";
    elements.poseStatus.classList.toggle("is-live", poseFresh);
    const pose = poseFromTelemetry(data, viewOrigin);
    elements.poseX.textContent = pose.position.x.toFixed(2);
    elements.poseY.textContent = pose.position.z.toFixed(2);
    elements.poseZ.textContent = pose.position.y.toFixed(2);
    elements.poseSpeed.textContent = speed.toFixed(2);
    elements.mapPoints.textContent = pointTotal.toLocaleString();
    elements.mapFrames.textContent = String(scanFrames.length);
    elements.droppedFrames.textContent = String(droppedFrames);
    updateClearanceUi(data);
    updateSourceUi("depth_camera", now);
    updateSourceUi("lidar", now);
  }

  function updateClearanceUi(data) {
    const obstacles = data?.obstacles ?? {};
    hardCgClearanceM = Math.max(
      0.01,
      finite(obstacles.hard_cg_clearance_m, hardCgClearanceM),
    );
    const timeoutMs =
      Math.max(0.01, finite(obstacles.source_stale_timeout_s, 0.25)) * 1000;
    const fresh =
      obstacles.age_ms !== null &&
      obstacles.age_ms !== undefined &&
      obstacles.age_ms <= timeoutMs;
    const rawStatus = fresh ? obstacles.clearance_status : "unknown";
    const status = ["clear", "breach"].includes(rawStatus)
      ? rawStatus
      : "unknown";
    const nearest = fresh ? obstacles.nearest_distance_m : null;
    const margin = fresh ? obstacles.clearance_margin_m : null;

    elements.clearancePanel.dataset.status = status;
    elements.clearanceStatus.textContent = status.toUpperCase();
    elements.clearanceLimit.textContent = `${hardCgClearanceM.toFixed(2)} m`;
    elements.clearanceNearest.textContent = Number.isFinite(nearest)
      ? `${nearest.toFixed(2)} m`
      : "-- m";
    elements.clearanceMargin.textContent = Number.isFinite(margin)
      ? `${margin >= 0 ? "+" : ""}${margin.toFixed(2)} m`
      : "-- m";

    const color =
      status === "breach" ? 0xf0655d : status === "clear" ? 0x8de06f : 0xf0b44b;
    clearanceBoundaryMaterial.color.setHex(color);
    clearanceZoneMaterial.color.setHex(color);
    clearanceZoneMaterial.opacity = status === "breach" ? 0.12 : 0.045;
  }

  function updateDrone(data, deltaSeconds) {
    if (!data) return;
    const pose = poseFromTelemetry(data, viewOrigin);
    const blend = 1 - Math.exp(-Math.min(deltaSeconds, 0.1) / 0.035);
    mapDrone.position.lerp(pose.position, blend);
    mapDrone.quaternion.slerp(pose.quaternion, blend);
    const cubeMount = data.cube_mount ?? {};
    mapCube.position.set(
      finite(cubeMount.x_m),
      -finite(cubeMount.z_m),
      finite(cubeMount.y_m),
    );
    mapCube.rotation.y = THREE.MathUtils.degToRad(
      finite(cubeMount.yaw_ccw_deg),
    );
    airframeRing.position.set(
      mapDrone.position.x,
      Math.max(0.012, mapDrone.position.y - 0.14),
      mapDrone.position.z,
    );
    clearanceZone.position.set(
      mapDrone.position.x,
      Math.max(0.01, mapDrone.position.y - 0.18),
      mapDrone.position.z,
    );
    clearanceBoundary.position.copy(clearanceZone.position);
    if (
      Math.abs(clearanceBoundary.geometry.parameters.outerRadius - hardCgClearanceM) >
      0.001
    ) {
      clearanceZone.geometry.dispose();
      clearanceZone.geometry = new THREE.CircleGeometry(hardCgClearanceM, 128);
      clearanceBoundary.geometry.dispose();
      clearanceBoundary.geometry = new THREE.RingGeometry(
        Math.max(0.01, hardCgClearanceM - 0.03),
        hardCgClearanceM,
        128,
      );
    }

    if (
      !trajectoryPoints.length ||
      trajectoryPoints[trajectoryPoints.length - 1].distanceTo(
        mapDrone.position,
      ) > MIN_DISPLAY_TRAJECTORY_STEP_M
    ) {
      trajectoryPoints.push(mapDrone.position.clone());
      if (trajectoryPoints.length > MAX_TRAIL_POINTS) trajectoryPoints.shift();
      updateTelemetryTrail(trajectoryPoints);
    }

    if (following) {
      if (!hasDronePosition) {
        lastDronePosition.copy(mapDrone.position);
        hasDronePosition = true;
      }
      const movement = mapDrone.position.clone().sub(lastDronePosition);
      camera.position.add(movement);
      controls.target.add(movement);
      controls.target.lerp(
        mapDrone.position.clone().add(new THREE.Vector3(0, 0.25, 0)),
        0.08,
      );
      lastDronePosition.copy(mapDrone.position);
    }
  }

  function render(frameTime, data, deltaSeconds) {
    if (!active) return;
    updateDrone(data, deltaSeconds);
    const nowSeconds = frameTime / 1000;
    const pointTotal = pruneFrames(nowSeconds);
    if (frameTime - lastUiUpdate > 120) {
      updateUi(data, pointTotal, performance.now());
      lastUiUpdate = frameTime;
    }
    controls.update();
    renderer.render(scene, camera);
  }

  elements.resetButton.addEventListener("click", resetCamera);
  elements.topButton.addEventListener("click", setTopView);
  elements.followButton.addEventListener("click", () => {
    following = !following;
    elements.followButton.classList.toggle("is-active", following);
    if (following) {
      lastDronePosition.copy(mapDrone.position);
      hasDronePosition = true;
    }
  });
  elements.trailButton.addEventListener("click", () => {
    rolling = !rolling;
    elements.trailButton.classList.toggle("is-active", rolling);
    elements.frameBadge.textContent = rolling
      ? "ROLLING LOCAL SCAN"
      : "LIVE FRAME";
    if (!rolling) {
      const retained = new Set();
      for (let index = scanFrames.length - 1; index >= 0; index -= 1) {
        const frame = scanFrames[index];
        if (!retained.has(frame.source)) {
          retained.add(frame.source);
          continue;
        }
        disposeFrame(frame);
        scanFrames.splice(index, 1);
      }
    }
  });
  elements.trajectoryButton.addEventListener("click", () => {
    navigationTrajectoriesVisible = !navigationTrajectoriesVisible;
    navigationTrajectories.visible = navigationTrajectoriesVisible;
    elements.trajectoryButton.classList.toggle(
      "is-active",
      navigationTrajectoriesVisible,
    );
    elements.trajectoryButton.setAttribute(
      "aria-pressed",
      String(navigationTrajectoriesVisible),
    );
    elements.spatialView.dataset.trajectories = navigationTrajectoriesVisible
      ? "on"
      : "off";
  });
  elements.clearButton.addEventListener("click", clearFrames);
  elements.pointSize.addEventListener("input", () => {
    pointSize = Number(elements.pointSize.value);
    elements.pointSizeValue.textContent = `${pointSize.toFixed(3)} m`;
    for (const frame of scanFrames) frame.object.material.size = pointSize;
  });
  elements.trailSeconds.addEventListener("input", () => {
    historySeconds = Number(elements.trailSeconds.value);
    elements.trailSecondsValue.textContent = `${historySeconds.toFixed(0)} s`;
  });
  elements.depthVisibility.addEventListener("change", () => {
    sourceState.depth_camera.visible = elements.depthVisibility.checked;
    if (!sourceState.depth_camera.visible) removeSourceFrames("depth_camera");
  });
  elements.lidarVisibility.addEventListener("change", () => {
    sourceState.lidar.visible = elements.lidarVisibility.checked;
    if (!sourceState.lidar.visible) removeSourceFrames("lidar");
  });
  controls.addEventListener("start", () => {
    following = false;
    elements.followButton.classList.remove("is-active");
  });

  elements.followButton.classList.add("is-active");
  elements.trailButton.classList.add("is-active");
  elements.spatialView.dataset.trajectories = "on";
  resetCamera();

  return {
    ingestTelemetry,
    render,
    resize,
    setActive(value) {
      active = value;
      if (active) {
        connectSpatialStream();
        connectTrajectoryStream();
        resize();
        renderer.render(scene, camera);
      } else {
        disconnectSpatialStream();
        disconnectTrajectoryStream();
      }
    },
  };
}

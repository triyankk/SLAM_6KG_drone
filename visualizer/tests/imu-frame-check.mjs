import assert from "node:assert/strict";
import * as THREE from "three";
import {
  mapIm10aDeltaToScene,
  mapIm10aVectorToScene,
} from "../src/imu-frame.js";

const tolerance = 1e-9;

function assertVector(actual, expected) {
  assert.ok(
    actual.distanceTo(expected) < tolerance,
    `${actual.toArray()} != ${expected.toArray()}`,
  );
}

function assertRotation(sensorAxis, sceneAxis) {
  const angle = 0.37;
  const sensorRotation = new THREE.Quaternion().setFromAxisAngle(
    sensorAxis,
    angle,
  );
  const mapped = mapIm10aDeltaToScene(
    new THREE.Quaternion(),
    sensorRotation,
  );
  const expected = new THREE.Quaternion().setFromAxisAngle(sceneAxis, angle);
  assert.ok(mapped.angleTo(expected) < tolerance);
}

assertVector(
  mapIm10aVectorToScene(new THREE.Vector3(), 1, 0, 0),
  new THREE.Vector3(0, 0, 1),
);
assertVector(
  mapIm10aVectorToScene(new THREE.Vector3(), 0, 1, 0),
  new THREE.Vector3(1, 0, 0),
);
assertVector(
  mapIm10aVectorToScene(new THREE.Vector3(), 0, 0, 1),
  new THREE.Vector3(0, 1, 0),
);

assertRotation(
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(1, 0, 0),
);
assertRotation(
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 0, 1),
);
assertRotation(
  new THREE.Vector3(0, 0, 1),
  new THREE.Vector3(0, 1, 0),
);

console.log("IM10A Y/X/-Z body-preview mapping verified");

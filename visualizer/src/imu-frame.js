import * as THREE from "three";

const sensorToScene = new THREE.Quaternion().setFromRotationMatrix(
  new THREE.Matrix4().set(
    0, 1, 0, 0,
    0, 0, 1, 0,
    1, 0, 0, 0,
    0, 0, 0, 1,
  ),
);
const sceneToSensor = sensorToScene.clone().invert();

export function mapIm10aDeltaToScene(target, sensorDelta) {
  return target
    .copy(sensorToScene)
    .multiply(sensorDelta)
    .multiply(sceneToSensor);
}

export function mapIm10aVectorToScene(target, x, y, z) {
  return target.set(y, z, x);
}

import * as THREE from "three";

export function box(width, height, depth, material) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(width, height, depth),
    material,
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

export function buildDrone() {
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
    opacity: 0.4,
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

  const cubeGroup = new THREE.Group();
  cubeGroup.name = "cube-orange";
  const cubeBody = box(
    0.12,
    0.035,
    0.1,
    new THREE.MeshStandardMaterial({
      color: 0xe8792f,
      emissive: 0x4d1c08,
      emissiveIntensity: 0.5,
      roughness: 0.38,
      metalness: 0.25,
    }),
  );
  const cubeArrow = box(
    0.045,
    0.01,
    0.018,
    new THREE.MeshStandardMaterial({
      color: 0xf7f8f7,
      emissive: 0x555b58,
      emissiveIntensity: 0.35,
    }),
  );
  cubeArrow.position.set(0.045, 0.023, 0);
  cubeGroup.add(cubeBody, cubeArrow);
  group.add(cubeGroup);

  const armLength = 0.82;
  const armA = box(armLength, 0.045, 0.055, frameMaterial);
  armA.rotation.y = Math.PI / 4;
  const armB = armA.clone();
  armB.rotation.y = -Math.PI / 4;
  group.add(armA, armB);

  const motorOffset = 0.425 / Math.sqrt(2);
  const motorPositions = [
    [motorOffset, 0.015, motorOffset],
    [motorOffset, 0.015, -motorOffset],
    [-motorOffset, 0.015, motorOffset],
    [-motorOffset, 0.015, -motorOffset],
  ];
  const motorGeometry = new THREE.CylinderGeometry(0.055, 0.062, 0.085, 24);
  const rotorGeometry = new THREE.RingGeometry(0.055, 0.2286, 64);
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

  const flowSensor = box(
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
  flowSensor.name = "hflow-sensor";
  flowSensor.position.set(0, -0.1, 0);
  group.add(flowSensor);

  const flowLens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.022, 0.027, 0.022, 24),
    new THREE.MeshStandardMaterial({
      color: 0x0b0d0c,
      roughness: 0.15,
      metalness: 0.45,
    }),
  );
  flowLens.position.set(0, -0.13, 0);
  group.add(flowLens);

  const depthCamera = box(
    0.045,
    0.045,
    0.12,
    new THREE.MeshStandardMaterial({
      color: 0x33474f,
      emissive: 0x10282f,
      emissiveIntensity: 0.55,
      roughness: 0.3,
      metalness: 0.45,
    }),
  );
  depthCamera.name = "depth-camera";
  depthCamera.position.set(0.19, -0.1, 0);
  group.add(depthCamera);

  const cameraLens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.014, 0.014, 0.012, 20),
    new THREE.MeshStandardMaterial({
      color: 0x111827,
      emissive: 0x204b67,
      emissiveIntensity: 0.7,
      roughness: 0.15,
    }),
  );
  cameraLens.rotation.z = -Math.PI / 2;
  cameraLens.position.set(0.219, -0.1, 0);
  group.add(cameraLens);

  const lidar = new THREE.Mesh(
    new THREE.CylinderGeometry(0.055, 0.055, 0.09, 28),
    new THREE.MeshStandardMaterial({
      color: 0x4f6b73,
      emissive: 0x16383d,
      emissiveIntensity: 0.6,
      roughness: 0.32,
      metalness: 0.5,
    }),
  );
  lidar.name = "jt16-sensor";
  lidar.position.set(0, 0.1, 0);
  lidar.castShadow = true;
  group.add(lidar);

  return group;
}

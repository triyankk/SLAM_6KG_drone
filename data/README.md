# Project Data

All generated flight-development data stays under this directory:

- `calibrations/`: measured intrinsics, extrinsics, biases, and noise profiles.
- `logs/`: application and readiness logs.
- `maps/`: versioned SLAM maps and map metadata.
- `recordings/`: ROS bags, MAVLink captures, and sensor datasets.

Passive field sessions are created under `recordings/flights/`. Each session is
self-describing and contains synchronized telemetry, shadow predictions,
RealSense/JT16 evidence, point clouds, optional Cube DataFlash logs, and
generated reports. See `../docs/FLIGHT_LOGGER.md`.

Do not read data from another workspace project. Import external datasets by
copying them into `recordings/` with provenance metadata.

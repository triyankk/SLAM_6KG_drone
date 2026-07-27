# Project Data

All generated flight-development data stays under this directory:

- `calibrations/`: measured intrinsics, extrinsics, biases, and noise profiles.
- `logs/`: application and readiness logs.
- `maps/`: versioned SLAM maps and map metadata.
- `recordings/`: ROS bags, MAVLink captures, and sensor datasets.

Do not read data from another workspace project. Import external datasets by
copying them into `recordings/` with provenance metadata.

# Host Hardware Configuration

This directory is the source of truth for host-level hardware rules needed by
the project. Installed copies under `/etc` are deployment artifacts; edit the
project copy first.

`udev/99-optflow-slam-usb-serial.rules` owns the stable JT16 and IM10A device
names.

The Hiwonder IM10A presents a CH340 USB serial bridge. Jetson Linux 36.5 does
not enable its CH341 kernel driver, so build and install the project-local
driver once:

```bash
./optflow build-ch341
sudo ./optflow install-ch341
```

After installation, reconnecting the IMU should create `/dev/imu_usb`
automatically. Kernel modules must not be loaded from another workspace folder.

The D415 uses the project-pinned librealsense `2.57.7` runtime. Install its
project-owned libusb permission rule once, then reconnect the camera:

```bash
sudo ./optflow install-realsense-rules
```

The rule is narrowed to the D415 USB product ID used by this aircraft. It is
based on the upstream librealsense device rule for the same SDK release.

The JT16 point-cloud path is serial RS485 through the Prolific `067b:23a3`
adapter. Build and install its driver once:

```bash
./optflow build-pl2303
sudo ./optflow install-pl2303
```

The udev rule binds adapter serial `DCCEb114J19` to `/dev/jt16_usb`. The JT16
manual specifies 3,000,000 baud for current firmware and 3,125,000 baud for
firmware `00.B0.1`; this unit must be probed before either value is marked
verified. Build the pinned official Hesai decoder and inspect the serial path:

```bash
./optflow build-jt16
./optflow lidar-status
```

The legacy `lidar-network` command is only an alias for `lidar-status`; it no
longer creates or modifies a NetworkManager profile.

With the aircraft disarmed, verify all three Jetson sensors:

```bash
./optflow sensor-check
```

`READY=true` requires a live IM10A stream, a sustained synchronized RGB-depth
stream, and valid JT16 serial packets. USB enumeration or one camera frame
alone is not a pass.

The Jetson headless display and VNC configuration is also kept here. Follow
[`docs/HEADLESS_ACCESS.md`](../docs/HEADLESS_ACCESS.md) before enclosing the
computer. The short form is:

```bash
x11vnc -storepasswd
sudo ./optflow install-headless-vnc
./optflow check-headless-vnc
```

The installer captures the current system configuration before replacing it.
It does not restart the display manager or reboot the Jetson.

The JT16 obstacle-avoidance-only runtime is a per-user service. Install it once
with:

```bash
./optflow install-flight-service
```

The installer enables user lingering so the runtime starts at boot without an
interactive login. It owns only the Cube MAVLink endpoint and JT16, publishes
paced proximity faces, and observes RC7 for alerts. Camera, IM10A, SLAM, and
companion movement output remain inactive. The service definition remains
project-owned at
`systemd/optflow-flight-logger.service`.

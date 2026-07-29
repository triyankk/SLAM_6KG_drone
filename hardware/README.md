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

Configure the dedicated JT16 Ethernet interface once:

```bash
./optflow lidar-network
```

This saves `192.168.1.100/24` on `enP8p1s0`, disables a default route, and adds
an explicit `192.168.1.201/32` host route. The host route prevents the Jetson's
Wi-Fi, which may also use `192.168.1.0/24`, from taking lidar traffic. The
profile activates automatically once Ethernet carrier is present.

With the aircraft disarmed, verify all three Jetson sensors:

```bash
./optflow sensor-check
```

`READY=true` requires a live IM10A stream, a sustained synchronized RGB-depth
stream, and JT16 UDP packets from the configured lidar address. USB
enumeration, Ethernet carrier, or one camera frame alone is not a pass.

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

The automatic flight logger is a per-user service. Install it once with:

```bash
./optflow install-flight-service
```

The installer enables user lingering so the logger starts at boot without an
interactive login. The service definition remains project-owned at
`systemd/optflow-flight-logger.service`.

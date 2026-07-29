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

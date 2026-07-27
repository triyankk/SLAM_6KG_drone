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

# CH341 USB Serial Driver

Jetson Linux 36.5 kernel `5.15.185-tegra` has `CONFIG_USB_SERIAL_CH341`
disabled. The Hiwonder IM10A uses a QinHeng CH340 interface with USB ID
`1a86:7523`, so no `/dev/ttyUSB*` device appears without this module.

`ch341.c` is the unmodified upstream Linux v5.15 driver:

- Source: `https://github.com/torvalds/linux/blob/v5.15/drivers/usb/serial/ch341.c`
- License: GPL-2.0, as declared by the source SPDX identifier.

Build it against the running Jetson kernel:

```bash
./optflow build-ch341
```

Installation changes host state and therefore requires operator-provided root
authentication:

```bash
sudo ./optflow install-ch341
```

The installer checks the module version before placing it under the running
kernel's `updates/optflow` directory. It also installs the project-owned udev
rule so the IM10A appears as `/dev/imu_usb`.

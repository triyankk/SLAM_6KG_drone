# PL2303 USB Serial Driver

Jetson Linux 36.5 kernel `5.15.185-tegra` does not provide the PL2303 module.
The Hesai JT16 connection box uses a Prolific PL2303GC USB-to-RS485 interface
with USB ID `067b:23a3`, so no serial device appears without this module.

`pl2303.c` and `pl2303.h` are based on upstream Linux v5.15 sources:

- Source: `https://github.com/torvalds/linux/tree/v5.15/drivers/usb/serial`
- The PL2303 device-detection block is backported from Linux v5.19:
  `https://github.com/torvalds/linux/blob/v5.19/drivers/usb/serial/pl2303.c`.
  That upstream change identifies USB revision `0x0705` as an HXN/G-series
  device; the JT16 adapter reports exactly that revision.
- License: GPL-2.0, as declared by the source SPDX identifiers.
- `pl2303.c` SHA-256:
  `2d68ed569d63dbb711a3b964ec420b634c4db184da6bd232fb1a82fa09237d73`
- `pl2303.h` SHA-256:
  `68a804a6a5e04f1abbbd9005c5df9fdb1ffeea412b2816dbefff3908521183a4`

Build it against the running Jetson kernel:

```bash
./optflow build-pl2303
```

Installation changes host state and requires operator-provided root
authentication:

```bash
sudo ./optflow install-pl2303
```

The installer validates the kernel version, installs the project udev rule,
loads the driver, and verifies `/dev/jt16_usb`.

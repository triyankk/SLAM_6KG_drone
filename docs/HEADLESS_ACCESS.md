# Jetson Headless Access

The VNC server mirrors the NVIDIA X display at `:0`. Merely allowing X to start
without a monitor does not guarantee that it creates a usable framebuffer. This
project supplies a validated 1920x1080 EDID and forces the known Jetson
connector `DFP-1` so the desktop also exists when the monitor is absent.

## Install

Create a VNC password as the normal `atas` user:

```bash
x11vnc -storepasswd
```

Install the X configuration and password-protected boot service:

```bash
sudo ./optflow install-headless-vnc
```

The installer validates both EDID checksums, preserves the original X and
systemd files, and deliberately does not restart the display manager or reboot.

## Prove Headless Boot

1. Shut down the Jetson cleanly.
2. Disconnect the physical display.
3. Cold boot the complete aircraft electronics.
4. Connect by SSH and VNC from another device on the same trusted LAN.
5. Run `./optflow check-headless-vnc`.
6. Repeat the cold-boot test three times before closing the electronics bay.

Direct VNC uses TCP port 5900. SSH on port 22 remains the recovery path when the
desktop fails. Do not expose either port to the public internet.

## Roll Back

From SSH or a local terminal:

```bash
sudo cp -a \
  /etc/X11/xorg.conf.before-optflow-headless \
  /etc/X11/xorg.conf
sudo systemctl reboot
```

If a JetPack or NVIDIA driver update changes display naming, connect a monitor
or a DisplayPort dummy plug, inspect `/var/log/Xorg.0.log`, and revalidate the
EDID configuration before flight.

## Before Enclosing the Jetson

- Pass three monitor-free cold boots with both SSH and VNC.
- Confirm the Cube UART, camera, IMU, lidar, and H-Flow survive the same boots.
- Retain physical access to power, recovery, USB, storage, and the cooling path.
- Keep a DisplayPort dummy plug available as the hardware recovery option.

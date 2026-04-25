#!/usr/bin/env python3

import argparse
import os
import select
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from jt16_serial_probe import PacketStats, choose_jt16_port, consume_packets, open_raw_serial


REPO_ROOT = Path(__file__).resolve().parents[1]
JT16_BAUD = 3000000
JT16_MANUAL_PATH = REPO_ROOT / "hardware" / "jt16_docs" / "JT16_User_Manual_J03-en-250420.pdf"
JT16_SIGNAL_SCAN_PATH = REPO_ROOT / "tools" / "jt16_signal_scan.py"
SERIAL_BOOT_INSTALLER_PATH = REPO_ROOT / "hardware" / "install_usb_serial_sensors_autostart.sh"
SERIAL_BOOT_RESTORE_PATH = REPO_ROOT / "hardware" / "enable_usb_serial_sensors.sh"
LOCAL_DRIVER_MODULES = {
    "pl2303": REPO_ROOT / "hardware" / "pl2303_module" / "pl2303.ko",
    "ch341": REPO_ROOT / "hardware" / "imu_module" / "ch341_module" / "ch341.ko",
}

KNOWN_USB_SERIAL_ADAPTERS = {
    ("067b", "23a3"): {
        "name": "Prolific USB-Serial Controller",
        "driver_module": "pl2303",
        "notes": "This repo currently sees your JT16 adapter on USB with this VID:PID.",
    },
    ("10c4", "ea60"): {
        "name": "Silicon Labs CP210x",
        "driver_module": "cp210x",
        "notes": "Supported by the stock Jetson kernel on this machine.",
    },
    ("0403", "6001"): {
        "name": "FTDI FT232",
        "driver_module": "ftdi_sio",
        "notes": "Supported by the stock Jetson kernel on this machine.",
    },
    ("1a86", "7523"): {
        "name": "QinHeng CH340",
        "driver_module": "ch341",
        "notes": "Common adapter, but the driver is not installed on this Jetson kernel.",
    },
}
SERIAL_NODE_PREFIXES = ("ttyUSB",)
JT16_PREFERRED_KEYS = {
    ("067b", "23a3"),
    ("10c4", "ea60"),
    ("0403", "6001"),
}


@dataclass
class UsbSerialAdapter:
    sysfs_path: Path
    vendor_id: str
    product_id: str
    manufacturer: str
    product: str
    serial_number: str
    tty_nodes: list[str]

    @property
    def key(self):
        return (self.vendor_id.lower(), self.product_id.lower())

    @property
    def known_info(self):
        return KNOWN_USB_SERIAL_ADAPTERS.get(self.key)

    @property
    def friendly_name(self) -> str:
        if self.known_info is not None:
            return self.known_info["name"]
        return f"{self.manufacturer or 'Unknown'} {self.product or 'USB serial adapter'}".strip()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validate a Hesai JT16 RS485 connection on Jetson. It checks whether the "
            "USB-RS485 adapter is visible, whether Linux created a serial node, and "
            "optionally probes for live JT16 packets."
        )
    )
    parser.add_argument(
        "--port",
        help="Explicit serial port to probe, for example /dev/ttyUSB0.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=JT16_BAUD,
        help="JT16 serial baud rate. Default: 3000000.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Seconds to listen for JT16 packets once a port is available.",
    )
    parser.add_argument(
        "--no-probe",
        action="store_true",
        help="Only validate USB/driver/port visibility, without opening the port.",
    )
    return parser.parse_args()


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore").strip()
    except OSError:
        return ""


def find_usb_serial_adapters() -> list[UsbSerialAdapter]:
    adapters: list[UsbSerialAdapter] = []
    for sysfs_path in sorted(Path("/sys/bus/usb/devices").iterdir()):
        if not (sysfs_path / "idVendor").exists():
            continue
        vendor_id = read_text(sysfs_path / "idVendor").lower()
        product_id = read_text(sysfs_path / "idProduct").lower()
        if not vendor_id or not product_id:
            continue

        tty_nodes: list[str] = []
        for interface in sorted(sysfs_path.glob(f"{sysfs_path.name}:*")):
            for tty_entry in sorted(interface.rglob("*")):
                if not tty_entry.name.startswith(SERIAL_NODE_PREFIXES):
                    continue
                dev_path = Path("/dev") / tty_entry.name
                if dev_path.exists():
                    tty_nodes.append(str(dev_path))

        product_text = read_text(sysfs_path / "product")
        manufacturer_text = read_text(sysfs_path / "manufacturer")
        is_known_adapter = (vendor_id, product_id) in KNOWN_USB_SERIAL_ADAPTERS
        looks_like_serial = "serial" in f"{manufacturer_text} {product_text}".lower()
        if not tty_nodes and not is_known_adapter and not looks_like_serial:
            continue

        adapters.append(
            UsbSerialAdapter(
                sysfs_path=sysfs_path,
                vendor_id=vendor_id,
                product_id=product_id,
                manufacturer=manufacturer_text,
                product=product_text,
                serial_number=read_text(sysfs_path / "serial"),
                tty_nodes=sorted(set(tty_nodes)),
            )
        )
    return adapters


def driver_module_present(module_name: str) -> bool:
    if (Path("/sys/module") / module_name).exists():
        return True
    modules_root = Path("/lib/modules") / os.uname().release / "kernel" / "drivers"
    return any(modules_root.rglob(f"{module_name}.ko*"))


def repo_local_module_present(module_name: str) -> bool:
    return LOCAL_DRIVER_MODULES.get(module_name, Path("/definitely/not/here")).exists()


def choose_probe_port(args, adapters: list[UsbSerialAdapter]) -> Optional[str]:
    if args.port:
        return args.port
    if Path("/dev/jt16_usb").exists():
        return "/dev/jt16_usb"
    preferred_nodes: list[str] = []
    for adapter in adapters:
        if adapter.key in JT16_PREFERRED_KEYS:
            preferred_nodes.extend(adapter.tty_nodes)
    for prefix in SERIAL_NODE_PREFIXES:
        for node in preferred_nodes:
            if Path(node).name.startswith(prefix):
                return node
    return None


def probe_jt16_packets(port: str, baud: int, duration_s: float) -> PacketStats:
    fd = open_raw_serial(port, baud)
    buffer = bytearray()
    stats = PacketStats()
    deadline_s = time.time() + max(duration_s, 0.2)
    try:
        while time.time() < deadline_s:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue
            chunk = os.read(fd, 8192)
            if not chunk:
                continue
            buffer.extend(chunk)
            consume_packets(buffer, stats)
    finally:
        os.close(fd)
    return stats


def print_adapter_summary(adapters: list[UsbSerialAdapter]):
    if JT16_MANUAL_PATH.exists():
        print(f"JT16 manual: {JT16_MANUAL_PATH}")
    else:
        print("JT16 manual: not bundled in this repo copy")
    print("Expected JT16 link: RS485, 3000000 8-N-1, white=DATA_A, green=DATA_B, supply=12-16V.")
    print()

    if not adapters:
        print("No USB serial adapters detected on the Jetson.")
        return

    print("Detected USB serial adapters:")
    for adapter in adapters:
        print(
            f"- {adapter.friendly_name}: "
            f"vid:pid={adapter.vendor_id}:{adapter.product_id} "
            f"serial={adapter.serial_number or 'n/a'}"
        )
        if adapter.tty_nodes:
            print(f"  tty nodes: {', '.join(adapter.tty_nodes)}")
        else:
            print("  tty nodes: none")
        if adapter.known_info is not None:
            module_name = adapter.known_info["driver_module"]
            module_state = "present" if driver_module_present(module_name) else "missing"
            if repo_local_module_present(module_name) and module_state == "missing":
                module_state = "repo-local build available"
            print(
                f"  expected kernel module: {module_name} "
                f"({module_state})"
            )
            print(f"  notes: {adapter.known_info['notes']}")
    print()


def print_no_tty_guidance(adapters: list[UsbSerialAdapter]):
    print("Result: the Jetson sees a USB-RS485 adapter, but Linux did not create /dev/ttyUSB*.")
    print()
    present_but_unbound = []
    for adapter in adapters:
        if adapter.known_info is None or adapter.tty_nodes:
            continue
        module_name = adapter.known_info["driver_module"]
        if not driver_module_present(module_name):
            print(
                f"- {adapter.friendly_name} ({adapter.vendor_id}:{adapter.product_id}) "
                f"needs the '{module_name}' kernel driver, but this Jetson kernel does not ship it."
            )
            if repo_local_module_present(module_name):
                print(
                    f"  Repo fix available: sudo bash {SERIAL_BOOT_INSTALLER_PATH}"
                )
        else:
            present_but_unbound.append(adapter)

    print()
    print("Fastest ways forward:")
    if any(repo_local_module_present(adapter.known_info["driver_module"]) for adapter in adapters if adapter.known_info):
        print(f"- Use the repo's permanent boot fix: sudo bash {SERIAL_BOOT_INSTALLER_PATH}")
        print(f"- For a one-shot restore right now: sudo bash {SERIAL_BOOT_RESTORE_PATH}")
    if present_but_unbound:
        print("- The driver is present, but the adapter still did not bind to a tty node.")
        print("- If this is the repo's Prolific JT16 adapter, reload the patched pl2303 module in hardware/pl2303_module and replug the adapter.")
    print("- Replace this USB-RS485 adapter with a CP210x or FTDI based adapter that the Jetson already supports.")
    print("- Or install/build the missing kernel driver as root, then reconnect the adapter and re-run this validator.")
    print()
    print("After a tty node appears, the existing raw probe is:")
    print(f"python3 {REPO_ROOT / 'tools' / 'jt16_serial_probe.py'} --port auto")


def print_probe_result(port: str, baud: int, stats: PacketStats):
    print(f"Probed {port} at {baud} baud.")
    print(
        "JT16 packets:"
        f" point={stats.point_packets}"
        f" imu={stats.imu_packets}"
        f" fault={stats.fault_packets}"
        f" sync_loss={stats.unknown_headers}"
    )
    if stats.point_packets > 0:
        print(
            "Stream status: OK"
            f" | azimuth={stats.last_azimuth_deg:.2f} deg"
            f" | dist_min/med/max={stats.last_min_distance_m:.2f}/"
            f"{stats.last_median_distance_m:.2f}/{stats.last_max_distance_m:.2f} m"
        )
        return

    print("Stream status: no JT16 point packets seen yet.")
    print("Check this next:")
    print("- The lidar is powered from 12 to 16 V.")
    print("- White wire is RS485 DATA_A and green wire is RS485 DATA_B.")
    print("- The RS485 adapter is connected to the lidar's RS485 pair, not the UART/GNSS pin.")
    print("- The lidar is connected to a host and should start streaming as soon as power and host link are both present.")
    print("- Run the multi-baud scanner to see whether any serial lane is alive at all:")
    print(f"  python3 {JT16_SIGNAL_SCAN_PATH} --port {port}")


def main():
    args = parse_args()
    adapters = find_usb_serial_adapters()
    print_adapter_summary(adapters)

    port = choose_probe_port(args, adapters)
    has_any_tty = any(adapter.tty_nodes for adapter in adapters)

    if port is None and not has_any_tty:
        print_no_tty_guidance(adapters)
        raise SystemExit(2)

    if args.no_probe:
        if port:
            print(f"Validation-only mode: candidate port is {port}")
        raise SystemExit(0)

    if port is None:
        raise SystemExit("No serial port available to probe. Use --port once /dev/ttyUSB* exists.")
    if not Path(port).exists():
        raise SystemExit(f"{port} does not exist.")

    stats = probe_jt16_packets(port, args.baud, args.duration)
    print_probe_result(port, args.baud, stats)
    if stats.point_packets <= 0:
        raise SystemExit(3)


if __name__ == "__main__":
    main()

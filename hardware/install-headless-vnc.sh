#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
EDID_HEX="${ROOT_DIR}/hardware/headless/optflow-headless-1080p.edid.hex"
XORG_SOURCE="${ROOT_DIR}/hardware/headless/xorg-headless.conf"
SERVICE_TEMPLATE="${ROOT_DIR}/hardware/systemd/x11vnc.service"
EDID_TARGET="/lib/firmware/edid/optflow-headless-1080p.bin"
XORG_TARGET="/etc/X11/xorg.conf"
SERVICE_TARGET="/etc/systemd/system/x11vnc.service"

if [[ "${EUID}" -ne 0 ]]; then
    printf 'Run this installer as root: sudo ./optflow install-headless-vnc\n' >&2
    exit 2
fi

TARGET_USER="${SUDO_USER:-atas}"
TARGET_UID="$(id -u "${TARGET_USER}")"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
VNC_PASSWORD_FILE="${TARGET_HOME}/.vnc/passwd"
XAUTHORITY_FILE="/run/user/${TARGET_UID}/gdm/Xauthority"

if [[ -z "${TARGET_HOME}" || ! -d "${TARGET_HOME}" ]]; then
    printf 'Cannot determine the home directory for user %s.\n' "${TARGET_USER}" >&2
    exit 2
fi

if [[ ! -r "${VNC_PASSWORD_FILE}" ]]; then
    printf '%s\n' \
        "VNC password file is missing: ${VNC_PASSWORD_FILE}" \
        "Run this as ${TARGET_USER}, then rerun the installer:" \
        "  x11vnc -storepasswd"
    exit 2
fi

for command in x11vnc xxd awk install sed systemctl; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        printf 'Required command is missing: %s\n' "${command}" >&2
        exit 2
    fi
done

temporary_edid="$(mktemp)"
temporary_service="$(mktemp)"
trap 'rm -f "${temporary_edid}" "${temporary_service}"' EXIT

xxd -r -p "${EDID_HEX}" "${temporary_edid}"

if [[ "$(wc -c < "${temporary_edid}")" -ne 256 ]]; then
    printf 'Refusing installation: decoded EDID is not 256 bytes.\n' >&2
    exit 1
fi

if [[ "$(xxd -p -l 8 "${temporary_edid}")" != "00ffffffffffff00" ]]; then
    printf 'Refusing installation: invalid EDID header.\n' >&2
    exit 1
fi

if ! od -An -tu1 -v "${temporary_edid}" | awk '
    {
        for (i = 1; i <= NF; i++) {
            sum += $i
            count++
            if (count % 128 == 0) {
                if (sum % 256 != 0) {
                    exit 1
                }
                sum = 0
            }
        }
    }
    END {
        if (count == 0 || count % 128 != 0) {
            exit 1
        }
    }
'; then
    printf 'Refusing installation: EDID checksum validation failed.\n' >&2
    exit 1
fi

timestamp="$(date +%Y%m%d-%H%M%S)"
if [[ -e "${XORG_TARGET}" ]]; then
    if [[ ! -e "${XORG_TARGET}.before-optflow-headless" ]]; then
        cp -a "${XORG_TARGET}" "${XORG_TARGET}.before-optflow-headless"
    fi
    cp -a "${XORG_TARGET}" "${XORG_TARGET}.backup-${timestamp}"
fi
if [[ -e "${SERVICE_TARGET}" ]]; then
    if [[ ! -e "${SERVICE_TARGET}.before-optflow-headless" ]]; then
        cp -a "${SERVICE_TARGET}" "${SERVICE_TARGET}.before-optflow-headless"
    fi
    cp -a "${SERVICE_TARGET}" "${SERVICE_TARGET}.backup-${timestamp}"
fi

install -D -m 0644 "${temporary_edid}" "${EDID_TARGET}"
install -m 0644 "${XORG_SOURCE}" "${XORG_TARGET}"

sed \
    -e "s|@XAUTHORITY@|${XAUTHORITY_FILE}|g" \
    -e "s|@VNC_PASSWORD_FILE@|${VNC_PASSWORD_FILE}|g" \
    "${SERVICE_TEMPLATE}" > "${temporary_service}"
install -m 0644 "${temporary_service}" "${SERVICE_TARGET}"

systemctl daemon-reload
systemctl enable x11vnc.service
systemctl restart x11vnc.service

printf '%s\n' \
    "Installed the headless 1920x1080 X configuration." \
    "Installed and restarted password-protected x11vnc." \
    "No display-manager restart or reboot was performed." \
    "" \
    "Next: power off, disconnect the monitor, cold boot, and run:" \
    "  ./optflow check-headless-vnc"

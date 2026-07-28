#!/usr/bin/env bash
set -u

DISPLAY_NAME="${DISPLAY:-:0}"
XAUTHORITY_FILE="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
failures=0

check() {
    local label="$1"
    shift
    if "$@" >/dev/null 2>&1; then
        printf 'PASS  %s\n' "${label}"
    else
        printf 'FAIL  %s\n' "${label}"
        failures=$((failures + 1))
    fi
}

check "x11vnc enabled at boot" systemctl is-enabled --quiet x11vnc.service
check "x11vnc service active" systemctl is-active --quiet x11vnc.service
check "VNC listening on TCP 5900" bash -c \
    'ss -ltn "sport = :5900" | grep -q LISTEN'
check "X authority file readable" test -r "${XAUTHORITY_FILE}"
check "X display ${DISPLAY_NAME} reachable" env \
    DISPLAY="${DISPLAY_NAME}" XAUTHORITY="${XAUTHORITY_FILE}" xdpyinfo
check "X framebuffer is 1920x1080" bash -c \
    "DISPLAY='${DISPLAY_NAME}' XAUTHORITY='${XAUTHORITY_FILE}' xdpyinfo 2>/dev/null | grep -Eq 'dimensions:[[:space:]]+1920x1080'"

if (( failures > 0 )); then
    printf '\n%d headless-access check(s) failed.\n' "${failures}" >&2
    exit 1
fi

printf '\nHeadless X and VNC checks passed.\n'

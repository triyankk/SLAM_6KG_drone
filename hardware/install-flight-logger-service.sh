#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="optflow-flight-logger.service"
SERVICE_SOURCE="${ROOT_DIR}/hardware/systemd/${SERVICE_NAME}"
USER_SERVICE_DIR="${HOME}/.config/systemd/user"
SERVICE_TARGET="${USER_SERVICE_DIR}/${SERVICE_NAME}"

if [[ ! -x "${ROOT_DIR}/.venv/bin/python" ]]; then
    printf 'Local environment is missing. Run: ./optflow setup\n' >&2
    exit 2
fi

for command in loginctl systemctl; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        printf 'Required command is missing: %s\n' "${command}" >&2
        exit 2
    fi
done

mkdir -p "${USER_SERVICE_DIR}"
ln -sfn "${SERVICE_SOURCE}" "${SERVICE_TARGET}"
systemctl --user daemon-reload
systemctl --user enable --now "${SERVICE_NAME}"

if loginctl enable-linger "${USER}"; then
    linger_result="enabled"
else
    linger_result="not enabled; the service will start at user login"
fi

printf '%s\n' \
    "Installed and started ${SERVICE_NAME}." \
    "Boot persistence: user lingering ${linger_result}." \
    "Status: ./optflow flight-status"

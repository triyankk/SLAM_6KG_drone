#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"
WHEEL_DIR="${ROOT_DIR}/vendor/python"
BOOTSTRAP_DIR="${ROOT_DIR}/runtime/bootstrap"
PYTHON="${VENV_DIR}/bin/python"

python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
architecture="$(uname -m)"

if [[ "${python_version}" != "3.10" || "${architecture}" != "aarch64" ]]; then
    printf '%s\n' \
        "The bundled runtime requires CPython 3.10 on aarch64." \
        "Detected Python ${python_version} on ${architecture}." >&2
    exit 2
fi

if [[ ! -d "${WHEEL_DIR}" ]]; then
    printf 'Bundled wheel directory is missing: %s\n' "${WHEEL_DIR}" >&2
    exit 2
fi

mkdir -p \
    "${ROOT_DIR}/data/calibrations" \
    "${ROOT_DIR}/data/logs" \
    "${ROOT_DIR}/data/maps" \
    "${ROOT_DIR}/data/recordings" \
    "${ROOT_DIR}/runtime" \
    "${ROOT_DIR}/ros_ws/src" \
    "${ROOT_DIR}/third_party"

if [[ ! -x "${PYTHON}" ]] || ! "${PYTHON}" -m pip --version >/dev/null 2>&1; then
    if ! python3 -m venv "${VENV_DIR}" >/dev/null 2>&1; then
        mkdir -p "${BOOTSTRAP_DIR}"
        python3 -m pip install \
            --disable-pip-version-check \
            --no-index \
            --find-links "${WHEEL_DIR}" \
            --requirement "${ROOT_DIR}/bootstrap-requirements.lock" \
            --target "${BOOTSTRAP_DIR}"
        PYTHONPATH="${BOOTSTRAP_DIR}" python3 -m virtualenv \
            --app-data "${ROOT_DIR}/runtime/virtualenv" \
            --no-download \
            --clear \
            "${VENV_DIR}"
    fi
fi

"${PYTHON}" -m pip install \
    --disable-pip-version-check \
    --no-index \
    --find-links "${WHEEL_DIR}" \
    --requirement "${ROOT_DIR}/requirements.lock"

if [[ ! -x "${ROOT_DIR}/visualizer/node_modules/.bin/vite" ]]; then
    printf '%s\n' \
        "Local frontend dependencies are missing." \
        "Restore visualizer/node_modules inside this project folder." >&2
    exit 2
fi

npm --prefix "${ROOT_DIR}/visualizer" run build

(
    cd "${ROOT_DIR}"
    "${PYTHON}" -m pytest
)

printf 'Project-local environment ready at %s\n' "${VENV_DIR}"

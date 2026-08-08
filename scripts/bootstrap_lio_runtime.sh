#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${ROOT_DIR}/runtime/lio"
CACHE="${RUNTIME}/cache"
SOURCE="${RUNTIME}/source"
BUILD="${RUNTIME}/build"
SYSROOT="${RUNTIME}/sysroot"
ROS_ROOT="${RUNTIME}/ros2"
PCL_PREFIX="${RUNTIME}/pcl"
PCL_ROS_PREFIX="${RUNTIME}/pcl_ros"
FAST_LIO_PREFIX="${RUNTIME}/fastlio"
TOOLS="${RUNTIME}/python-tools"
MARKER="${RUNTIME}/runtime-manifest.json"

ROS_TAG="release-humble-20260220"
ROS_ARCHIVE="ros2-humble-20260220-linux-jammy-arm64.tar.bz2"
ROS_URL="https://github.com/ros2/ros2/releases/download/${ROS_TAG}/${ROS_ARCHIVE}"
ROS_SHA256="094b499f618d673d154a35784db4a94be4cd5701c43ee95e5d70551d08d21faa"
PCL_REVISION="e8ed4be802f7d0b1acff2f8b01d7c5f381190e05"
PCL_MSGS_REVISION="8a925a7c4626df52dba7ccc5bda5900d63678880"
PCL_CONVERSIONS_REVISION="67a5c2ba4c4de3ca21c5cd495812a01ced3fb69a"
FAST_LIO_REVISION="bb2842d34990761eebbd4cc3188e94c7c662a673"
IKD_TREE_REVISION="e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4"
CMAKE="${ROOT_DIR}/.venv/bin/cmake"
COLCON="${TOOLS}/bin/colcon"

if [[ "$(uname -m)" != "aarch64" ]]; then
    printf '%s\n' "The pinned LIO runtime supports this Jetson's aarch64 platform." >&2
    exit 2
fi
if [[ ! -x "${CMAKE}" ]]; then
    printf 'Native project CMake is missing. Run: %s/optflow setup\n' \
        "${ROOT_DIR}" >&2
    exit 2
fi
if [[ -f "${MARKER}" ]] \
    && grep -q "${FAST_LIO_REVISION}" "${MARKER}" \
    && [[ -x "${FAST_LIO_PREFIX}/fast_lio/lib/fast_lio/fastlio_mapping" ]]; then
    printf 'Pinned LIO runtime is already ready at %s\n' "${RUNTIME}"
    exit 0
fi

mkdir -p \
    "${CACHE}" \
    "${SOURCE}" \
    "${BUILD}" \
    "${SYSROOT}" \
    "${ROS_ROOT}" \
    "${PCL_PREFIX}" \
    "${PCL_ROS_PREFIX}" \
    "${FAST_LIO_PREFIX}" \
    "${TOOLS}"

if [[ ! -f "${CACHE}/${ROS_ARCHIVE}" ]]; then
    curl --fail --location --retry 4 --output "${CACHE}/${ROS_ARCHIVE}" \
        "${ROS_URL}"
fi
printf '%s  %s\n' "${ROS_SHA256}" "${CACHE}/${ROS_ARCHIVE}" \
    | sha256sum --check -
if [[ ! -f "${ROS_ROOT}/setup.bash" ]]; then
    tar -xjf "${CACHE}/${ROS_ARCHIVE}" \
        --strip-components=1 \
        -C "${ROS_ROOT}"
fi

packages=(
    libboost1.74-dev
    libboost-date-time1.74-dev
    libboost-date-time1.74.0
    libboost-filesystem1.74-dev
    libboost-filesystem1.74.0
    libboost-iostreams1.74-dev
    libboost-iostreams1.74.0
    libboost-regex1.74-dev
    libboost-regex1.74.0
    libboost-serialization1.74-dev
    libboost-serialization1.74.0
    libboost-system1.74-dev
    libboost-system1.74.0
    libconsole-bridge-dev
    libconsole-bridge1.0
    libeigen3-dev
    libflann-dev
    libflann1.9
    libfmt-dev
    libfmt8
    liblz4-dev
    libspdlog-dev
    libspdlog1
)
package_cache="${CACHE}/ubuntu-packages"
mkdir -p "${package_cache}"
(
    cd "${package_cache}"
    apt-get download "${packages[@]}"
)
for package in "${package_cache}"/*.deb; do
    dpkg-deb -x "${package}" "${SYSROOT}"
done

if [[ ! -x "${COLCON}" ]] \
    || ! PYTHONPATH="${TOOLS}" python3 -c \
        'import em, lark; assert em.__version__ == "3.3.4"; assert lark.__version__ == "1.1.9"' \
        >/dev/null 2>&1; then
    python3 -m pip install \
        --disable-pip-version-check \
        --target "${TOOLS}" \
        --upgrade \
        "colcon-common-extensions==0.3.0"
    python3 -m pip install \
        --disable-pip-version-check \
        --target "${TOOLS}" \
        --upgrade \
        --force-reinstall \
        "empy==3.3.4" \
        "lark==1.1.9"
fi

if [[ ! -d "${SOURCE}/pcl/.git" ]]; then
    git clone https://github.com/PointCloudLibrary/pcl.git "${SOURCE}/pcl"
fi
git -C "${SOURCE}/pcl" checkout --detach "${PCL_REVISION}"

export LIBRARY_PATH="${SYSROOT}/usr/lib/aarch64-linux-gnu"
export LD_LIBRARY_PATH="${PCL_PREFIX}/lib:${SYSROOT}/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="${PCL_PREFIX}:${SYSROOT}/usr:${CMAKE_PREFIX_PATH:-}"
export PKG_CONFIG_PATH="${PCL_PREFIX}/lib/pkgconfig:${SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"

if [[ ! -f "${PCL_PREFIX}/lib/libpcl_filters.so" ]]; then
    "${CMAKE}" -S "${SOURCE}/pcl" -B "${BUILD}/pcl" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${PCL_PREFIX}" \
        -DCMAKE_PREFIX_PATH="${SYSROOT}/usr" \
        -DEigen3_DIR="${SYSROOT}/usr/share/eigen3/cmake" \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_apps=OFF \
        -DBUILD_benchmarks=OFF \
        -DBUILD_examples=OFF \
        -DBUILD_global_tests=OFF \
        -DBUILD_common=ON \
        -DBUILD_filters=ON \
        -DBUILD_io=ON \
        -DBUILD_kdtree=ON \
        -DBUILD_octree=ON \
        -DBUILD_sample_consensus=ON \
        -DBUILD_search=ON \
        -DBUILD_2d=OFF \
        -DBUILD_features=OFF \
        -DBUILD_geometry=OFF \
        -DBUILD_keypoints=OFF \
        -DBUILD_ml=OFF \
        -DBUILD_outofcore=OFF \
        -DBUILD_people=OFF \
        -DBUILD_recognition=OFF \
        -DBUILD_registration=OFF \
        -DBUILD_segmentation=OFF \
        -DBUILD_simulation=OFF \
        -DBUILD_stereo=OFF \
        -DBUILD_surface=OFF \
        -DBUILD_tools=OFF \
        -DBUILD_tracking=OFF \
        -DBUILD_visualization=OFF \
        -DWITH_CUDA=OFF \
        -DWITH_LIBUSB=OFF \
        -DWITH_OPENGL=OFF \
        -DWITH_OPENNI=OFF \
        -DWITH_OPENNI2=OFF \
        -DWITH_PCAP=OFF \
        -DWITH_PNG=OFF \
        -DWITH_QHULL=OFF \
        -DWITH_QT=OFF \
        -DWITH_RSSDK2=OFF \
        -DWITH_VTK=OFF
    "${CMAKE}" --build "${BUILD}/pcl" --parallel 1
    "${CMAKE}" --install "${BUILD}/pcl"
fi

if [[ ! -d "${SOURCE}/pcl_msgs/.git" ]]; then
    git clone --branch ros2 \
        https://github.com/ros-perception/pcl_msgs.git \
        "${SOURCE}/pcl_msgs"
fi
git -C "${SOURCE}/pcl_msgs" checkout --detach "${PCL_MSGS_REVISION}"
if [[ ! -d "${SOURCE}/perception_pcl/.git" ]]; then
    git clone --branch humble \
        https://github.com/ros-perception/perception_pcl.git \
        "${SOURCE}/perception_pcl"
fi
git -C "${SOURCE}/perception_pcl" checkout --detach \
    "${PCL_CONVERSIONS_REVISION}"

# shellcheck disable=SC1091
set +u
source "${ROS_ROOT}/setup.bash"
set -u
export PYTHONPATH="${TOOLS}:${PYTHONPATH:-}"
export CMAKE_COMMAND="${CMAKE}"

"${COLCON}" --log-base "${BUILD}/pcl_ros_log" build \
    --base-paths "${SOURCE}/pcl_msgs" "${SOURCE}/perception_pcl" \
    --build-base "${BUILD}/pcl_ros" \
    --install-base "${PCL_ROS_PREFIX}" \
    --packages-select pcl_msgs pcl_conversions \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPCL_DIR="${PCL_PREFIX}/share/pcl-1.12" \
    -DEigen3_DIR="${SYSROOT}/usr/share/eigen3/cmake" \
    -Dspdlog_DIR="${SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/spdlog" \
    -Dfmt_DIR="${SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/fmt"

if [[ ! -d "${SOURCE}/FAST_LIO_Hesai/.git" ]]; then
    git clone --branch ROS2 \
        https://github.com/HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai.git \
        "${SOURCE}/FAST_LIO_Hesai"
fi
git -C "${SOURCE}/FAST_LIO_Hesai" checkout --detach "${FAST_LIO_REVISION}"
git -C "${SOURCE}/FAST_LIO_Hesai" submodule update --init --recursive
actual_ikd_revision="$(
    git -C "${SOURCE}/FAST_LIO_Hesai/include/ikd-Tree" rev-parse HEAD
)"
if [[ "${actual_ikd_revision}" != "${IKD_TREE_REVISION}" ]]; then
    printf 'Unexpected ikd-Tree revision: %s\n' "${actual_ikd_revision}" >&2
    exit 2
fi
patch_file="${ROOT_DIR}/third_party/patches/fast_lio_hesai-user-runtime.patch"
if git -C "${SOURCE}/FAST_LIO_Hesai" apply --check "${patch_file}" \
    >/dev/null 2>&1; then
    git -C "${SOURCE}/FAST_LIO_Hesai" apply "${patch_file}"
fi

export AMENT_PREFIX_PATH="${PCL_ROS_PREFIX}/pcl_conversions:${PCL_ROS_PREFIX}/pcl_msgs:${AMENT_PREFIX_PATH:-}"
"${COLCON}" --log-base "${BUILD}/fastlio_log" build \
    --base-paths "${SOURCE}/FAST_LIO_Hesai" \
    --build-base "${BUILD}/fastlio" \
    --install-base "${FAST_LIO_PREFIX}" \
    --packages-select fast_lio \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPCL_DIR="${PCL_PREFIX}/share/pcl-1.12" \
    -DEigen3_DIR="${SYSROOT}/usr/share/eigen3/cmake" \
    -Dspdlog_DIR="${SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/spdlog" \
    -Dfmt_DIR="${SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/fmt" \
    -Dconsole_bridge_DIR="${SYSROOT}/usr/lib/aarch64-linux-gnu/console_bridge/cmake"

printf '%s\n' \
    '{' \
    '  "schema_version": 1,' \
    "  \"ros_tag\": \"${ROS_TAG}\"," \
    "  \"pcl_revision\": \"${PCL_REVISION}\"," \
    "  \"pcl_msgs_revision\": \"${PCL_MSGS_REVISION}\"," \
    "  \"pcl_conversions_revision\": \"${PCL_CONVERSIONS_REVISION}\"," \
    "  \"fast_lio_revision\": \"${FAST_LIO_REVISION}\"," \
    "  \"ikd_tree_revision\": \"${IKD_TREE_REVISION}\"" \
    '}' > "${MARKER}"

printf 'Pinned JT16 FAST-LIO2 runtime is ready at %s\n' "${RUNTIME}"

#!/usr/bin/env bash
# =============================================================================
# app/slam2d_nodelet/make.sh — 2D 激光 SLAM nodelet（无 catkin）
# -----------------------------------------------------------------------------
#   ./make.sh x86       编译 libslam2d_nodelet.so → build/
#   ./make.sh install   安装到 ../../app_runtime/lib/
#   ./make.sh clean
#
# 依赖：custom_mini_install（./custom_mini.sh build，含 setup_eigen 提供的 Eigen 头文件）。
# 注册：app_runtime/plugins.json 手工维护，install 时若未注册会打印条目提示。
# =============================================================================
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${PKG}/../.." && pwd)"
ROS_INSTALL="${ROS_INSTALL:-${ROOT}/custom_mini_install}"
RUNTIME="${ROOT}/app_runtime"
BUILD="${PKG}/build"
JOBS="$(nproc 2>/dev/null || echo 4)"
PY_SITE="${ROS_INSTALL}/lib/python3.12/site-packages"
PY_COMPAT="${ROOT}/python_compat"
SO_NAME="libslam2d_nodelet.so"

fail() { echo "ERROR: $*" >&2; exit 1; }

cmd_help() {
  cat <<EOF
用法: $0 <x86|install|clean|help>
  x86       编译 ${SO_NAME}（ROS_INSTALL=${ROS_INSTALL}）
  install   安装到 ${RUNTIME}/lib/
  clean     删除 ${BUILD}
EOF
}

prepare_env() {
  [[ -f "${ROS_INSTALL}/lib/libnodeletlib.so" ]] || fail "缺少 ROS 环境: ${ROS_INSTALL}（请先 ./custom_mini.sh build）"
  [[ -f "${ROS_INSTALL}/include/eigen3/Eigen/Core" ]] || fail "缺少 Eigen 头文件，请重跑 ./custom_mini.sh build（setup_eigen 步骤）"
  export CMAKE_PREFIX_PATH="${ROS_INSTALL}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${ROS_INSTALL}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="${ROS_INSTALL}/bin:${PATH}"
  export PYTHONPATH="${PY_COMPAT}:${PY_SITE}:${ROS_INSTALL}/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
  export BOOST_ROOT="${ROS_INSTALL}"
  export Boost_NO_SYSTEM_PATHS=ON
  export PYTHONNOUSERSITE=1
}

find_so() {
  local f
  for f in "${BUILD}/${SO_NAME}" "${BUILD}/lib/${SO_NAME}"; do
    [[ -f "${f}" ]] && { printf '%s\n' "${f}"; return 0; }
  done
  f="$(find "${BUILD}" -name "${SO_NAME}" -type f 2>/dev/null | head -n 1 || true)"
  [[ -n "${f}" ]] || return 1
  printf '%s\n' "${f}"
}

cmd_x86() {
  prepare_env
  echo "==== slam2d_nodelet x86 build ===="
  mkdir -p "${BUILD}"
  local extra_flags="-I${ROS_INSTALL}/include -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DBOOST_NO_CXX98_FUNCTION_BASE -Wno-deprecated-declarations -fpermissive"
  cmake -S "${PKG}" -B "${BUILD}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${ROS_INSTALL}" \
    -DCMAKE_INSTALL_RPATH="" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=OFF \
    -DCMAKE_SKIP_INSTALL_RPATH=ON \
    -DCMAKE_CXX_STANDARD=14 \
    "-DCMAKE_CXX_FLAGS=${extra_flags}"
  cmake --build "${BUILD}" -- -j "${JOBS}"
  local so; so="$(find_so)" || fail "未生成 ${SO_NAME}"
  echo "DONE: ${so}"
  echo "下一步: $0 install"
}

cmd_install() {
  local so; so="$(find_so)" || fail "缺少 ${SO_NAME}，请先: $0 x86"
  mkdir -p "${RUNTIME}/lib"
  cp -a "${so}" "${RUNTIME}/lib/${SO_NAME}"
  echo "INSTALLED: ${RUNTIME}/lib/${SO_NAME}"
  if ! grep -q "${SO_NAME}" "${RUNTIME}/plugins.json" 2>/dev/null; then
    echo "WARN: ${RUNTIME}/plugins.json 尚未注册 slam2d，请手工加入以下条目:" >&2
    cat >&2 <<'EOF'
    {
      "name": "/slam2d",
      "class": "slam2d_nodelet::Slam2dNodelet",
      "library": "libslam2d_nodelet.so",
      "enabled": true,
      "remap": { "scan": "/mycar/base_scan", "odom": "/mycar/odom", "reset": "/slam2d/reset" },
      "args": []
    }
EOF
  fi
}

cmd_clean() { rm -rf "${BUILD}"; echo "删除 ${BUILD}"; }

case "${1:-help}" in
  x86) cmd_x86 ;;
  install) cmd_install ;;
  clean) cmd_clean ;;
  help|-h|--help) cmd_help ;;
  *) echo "未知命令: $1" >&2; cmd_help; exit 1 ;;
esac

#!/usr/bin/env bash
# =============================================================================
# app/custom_ros_nodelet/make.sh
# -----------------------------------------------------------------------------
# 编译 JSON 驱动的 custom_ros_nodelet_manager。依赖仓库根目录 custom_mini_install/。
# nlohmann/json.hpp 使用本包 include/，缺失则编译失败。
#
#   ./make.sh x86       编译（中间文件在本目录 build/）
#   ./make.sh install   安装可执行文件 → ../../app_runtime/bin/
#   ./make.sh clean     删除 build/
#   ./make.sh help
# =============================================================================
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${PKG}/../.." && pwd)"
ROS_INSTALL="${ROOT}/custom_mini_install"
RUNTIME="${ROOT}/app_runtime"
BUILD="${PKG}/build"
DOC="${ROOT}/doc/app"
BIN_NAME="custom_ros_nodelet_manager"
JOBS="$(nproc 2>/dev/null || echo 4)"
PY_SITE="${ROS_INSTALL}/lib/python3.12/site-packages"
PY_COMPAT="${ROOT}/python_compat"

fail() { echo "ERROR: $*" >&2; exit 1; }

cmd_help() {
  cat <<EOF
用法: $0 <x86|install|clean|help>

  x86       针对 x86_64 编译 ${BIN_NAME} → ${BUILD}/
            前提: ${ROS_INSTALL}（./custom_mini.sh build）
            头文件: ${PKG}/include/nlohmann/json.hpp

  install   将 ${BIN_NAME} 拷到 ${RUNTIME}/bin/
            并确保 app_runtime 有 run.sh / plugins.json 软链接
            前提: 已执行 x86

  clean     删除 ${BUILD}

  help      显示本说明
EOF
}

prepare_env() {
  [[ -f "${ROS_INSTALL}/lib/libnodeletlib.so" ]] || fail "缺少 ROS 环境，请先: ./custom_mini.sh build"
  export CMAKE_PREFIX_PATH="${ROS_INSTALL}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${ROS_INSTALL}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="${ROS_INSTALL}/bin:${PATH}"
  export PYTHONPATH="${PY_COMPAT}:${PY_SITE}:${ROS_INSTALL}/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
  export BOOST_ROOT="${ROS_INSTALL}"
  export Boost_NO_SYSTEM_PATHS=ON
  export PYTHONNOUSERSITE=1
}

cmd_x86() {
  prepare_env
  echo "==== custom_ros_nodelet x86 build ===="
  mkdir -p "${BUILD}"
  cmake -S "${PKG}" -B "${BUILD}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${ROS_INSTALL}" \
    -DCMAKE_INSTALL_RPATH="" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=OFF \
    -DCMAKE_SKIP_INSTALL_RPATH=ON \
    -DCMAKE_CXX_STANDARD=14
  cmake --build "${BUILD}" -- -j "${JOBS}"
  [[ -x "${BUILD}/${BIN_NAME}" ]] || fail "未生成 ${BUILD}/${BIN_NAME}"
  echo "DONE: ${BUILD}/${BIN_NAME}"
  echo "下一步: $0 install"
}

ensure_runtime_docs() {
  mkdir -p "${RUNTIME}/bin" "${RUNTIME}/lib"
  [[ -e "${DOC}/run.sh" ]] || fail "缺少 ${DOC}/run.sh"
  [[ -e "${DOC}/plugins.json" ]] || fail "缺少 ${DOC}/plugins.json"
  ln -sfn "../doc/app/run.sh" "${RUNTIME}/run.sh"
  ln -sfn "../doc/app/plugins.json" "${RUNTIME}/plugins.json"
  ln -sfn "../doc/app/README.md" "${RUNTIME}/README.md"
  chmod +x "${DOC}/run.sh"
}

cmd_install() {
  local bin="${BUILD}/${BIN_NAME}"
  [[ -x "${bin}" ]] || fail "缺少 ${bin}，请先: $0 x86"
  ensure_runtime_docs
  rm -f "${RUNTIME}/bin/custom_mini_manager"
  cp -a "${bin}" "${RUNTIME}/bin/${BIN_NAME}"
  echo "INSTALLED: ${RUNTIME}/bin/${BIN_NAME}"
}

cmd_clean() {
  echo "删除 ${BUILD}"
  rm -rf "${BUILD}"
}

case "${1:-help}" in
  x86) cmd_x86 ;;
  install) cmd_install ;;
  clean) cmd_clean ;;
  help|-h|--help) cmd_help ;;
  *) echo "未知命令: $1" >&2; cmd_help; exit 1 ;;
esac

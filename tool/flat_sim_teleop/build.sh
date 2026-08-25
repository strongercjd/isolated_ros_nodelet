#!/usr/bin/env bash
# =============================================================================
# tool/flat_sim_teleop/build.sh —— 独立编译 flat_sim 键盘遥控
# -----------------------------------------------------------------------------
# 与仓库 src/ 无关：只依赖隔离 ROS（custom_mini_install）。
#
# 子命令（无参数 = build）：
#   ./build.sh deps   检查并安装系统依赖
#   ./build.sh build  CMake 配置 → 编译 → 安装到 install/
#   ./build.sh clean  删除 build/ 与 install/
#   ./build.sh help   参数说明
#
# 产物：install/bin/flat_sim_teleop
# 用法：先跑 flat_sim，再 ./run_teleop.sh
# =============================================================================
set -euo pipefail

TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$(dirname "${TOOL}")")"

ROS_INSTALL="${REPO}/custom_mini_install"
PREFIX="${TOOL}/install"
BUILD="${TOOL}/build"
BDIR="${BUILD}/teleop"
LOG="${BUILD}/build.log"

JOBS="$(nproc 2>/dev/null || echo 4)"

log()  { echo "[flat_sim_teleop/build.sh] $*"; }
fail() { echo "错误：$*" >&2; exit 1; }

prepare_env() {
  [[ -d "${ROS_INSTALL}" && -f "${ROS_INSTALL}/setup.bash" ]] \
    || fail "缺少隔离 ROS（${ROS_INSTALL}）。请先在仓库根目录执行 ./custom_mini.sh build"
  command -v cmake >/dev/null || fail "缺少 cmake。请先 ./build.sh deps"
  command -v g++   >/dev/null || fail "缺少 g++。请先 ./build.sh deps"
  command -v make  >/dev/null || fail "缺少 make。请先 ./build.sh deps"

  export CMAKE_PREFIX_PATH="${ROS_INSTALL}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export PKG_CONFIG_PATH="${ROS_INSTALL}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
  export LD_LIBRARY_PATH="${ROS_INSTALL}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="${ROS_INSTALL}/bin:${PATH}"
  local pyver
  pyver="$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0], sys.version_info[1]))')"
  export PYTHONPATH="${REPO}/python_compat:${ROS_INSTALL}/lib/python${pyver}/site-packages:${ROS_INSTALL}/local/lib/python${pyver}/dist-packages:${ROS_INSTALL}/lib/python3/dist-packages:${ROS_INSTALL}/lib/python${pyver}/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
  export ROS_PYTHON_VERSION=3
  export PYTHONNOUSERSITE=1
  export BOOST_ROOT="${ROS_INSTALL}"
  export Boost_NO_SYSTEM_PATHS=ON
}

deps() {
  local pkgs=(build-essential cmake)
  local miss=() p
  for p in "${pkgs[@]}"; do
    dpkg -s "${p}" >/dev/null 2>&1 || miss+=("${p}")
  done
  if [[ ${#miss[@]} -eq 0 ]]; then
    log "系统依赖已全部安装，无需 sudo。"
    return 0
  fi
  echo "需要 sudo 安装以下系统包："
  printf '  %s\n' "${miss[@]}"
  sudo apt-get update
  sudo apt-get install -y "${miss[@]}"
  log "系统依赖安装完成。"
}

do_build() {
  [[ -d "${ROS_INSTALL}" ]] || fail "缺少隔离 ROS。请先 ./custom_mini.sh build"
  if ! rm -rf "${BDIR}" 2>/dev/null; then
    fail "无法清理 ${BDIR}。请先：sudo chown -R \"\$USER:\$USER\" \"${BUILD}\""
  fi
  mkdir -p "${BUILD}" "${PREFIX}"
  prepare_env

  log "CMake 配置 -> ${BDIR}（日志：${LOG}）"
  cmake -S "${TOOL}" -B "${BDIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_INSTALL_RPATH="${ROS_INSTALL}/lib" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON 2>&1 | tee "${LOG}"

  log "编译（-j${JOBS}）"
  cmake --build "${BDIR}" -- -j "${JOBS}" 2>&1 | tee -a "${LOG}"

  log "安装 -> ${PREFIX}"
  cmake --install "${BDIR}" 2>&1 | tee -a "${LOG}"

  log "完成：${PREFIX}/bin/flat_sim_teleop"
  log "运行：./tool/flat_sim_teleop/run_teleop.sh（需 flat_sim 已在运行）"
}

clean() {
  rm -rf "${BUILD}" "${PREFIX}"
  log "已删除 build/ 与 install/"
}

help() {
  sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

cmd="${1:-build}"
case "${cmd}" in
  deps)  deps ;;
  build) do_build ;;
  clean) clean ;;
  help|-h|--help) help ;;
  *) echo "未知子命令：${cmd}" >&2; help >&2; exit 1 ;;
esac

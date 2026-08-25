#!/usr/bin/env bash
# =============================================================================
# tool/flat_sim/build.sh —— 独立编译 flat_sim（自研 2D 机器人仿真，C++ / CMake）
# -----------------------------------------------------------------------------
# 与仓库 src/ 无关：只依赖「隔离 ROS 环境」（custom_mini_install，由仓库根目录
# ./custom_mini.sh build 生成）。产物只写在本工具 build/ 与 install/。
#
# 子命令（无参数 = build）：
#   ./build.sh deps   检查并安装系统依赖（需要 sudo，见本工具 README.md）
#   ./build.sh build  CMake 配置 → 编译 → 安装到 install/（默认）
#   ./build.sh clean  删除 build/ 与 install/（保留源码 src/、worlds/、models/）
#   ./build.sh help   参数说明
#
# 产物：
#   install/bin/flat_sim_node               可执行文件
#   install/share/flat_sim/worlds|models/   示例世界与机器人模型
#   build/                                  中间文件；日志 build/build.log
#
# 编完运行：./run_flat_sim.sh（GUI）或 ./run_flat_sim.sh --headless
# =============================================================================
set -euo pipefail

TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../tool/flat_sim
REPO="$(dirname "$(dirname "${TOOL}")")"               # 仓库根（tool/flat_sim 上溯两级）

ROS_INSTALL="${REPO}/custom_mini_install"        # 隔离 ROS（custom_mini.sh build 生成）
PREFIX="${TOOL}/install"
BUILD="${TOOL}/build"
BDIR="${BUILD}/sim"                              # CMake 构建目录
LOG="${BUILD}/build.log"

JOBS="$(nproc 2>/dev/null || echo 4)"

# -----------------------------------------------------------------------------
# 小工具
# -----------------------------------------------------------------------------
log()  { echo "[flat_sim/build.sh] $*"; }
fail() { echo "错误：$*" >&2; exit 1; }

# 编译前注入隔离环境（ROS / Boost / Python 全部指向仓库内目录；不要用 sudo 跑 build）
prepare_env() {
  [[ -d "${ROS_INSTALL}" && -f "${ROS_INSTALL}/setup.bash" ]] \
    || fail "缺少隔离 ROS（${ROS_INSTALL}）。请先在仓库根目录执行 ./custom_mini.sh build"
  command -v cmake >/dev/null || fail "缺少 cmake。请先 ./build.sh deps 安装系统依赖"
  command -v g++   >/dev/null || fail "缺少 g++。请先 ./build.sh deps 安装系统依赖"
  command -v make  >/dev/null || fail "缺少 make。请先 ./build.sh deps 安装系统依赖"

  export CMAKE_PREFIX_PATH="${ROS_INSTALL}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export PKG_CONFIG_PATH="${ROS_INSTALL}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
  export LD_LIBRARY_PATH="${ROS_INSTALL}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="${ROS_INSTALL}/bin:${PATH}"
  # catkin 系 Config.cmake 在 configure 阶段要 import em / catkin_pkg 等，
  # PYTHONPATH 必须指向隔离 ROS 的 python 包目录（与 stage_ros 的 build.sh 相同）。
  local pyver
  pyver="$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0], sys.version_info[1]))')"
  export PYTHONPATH="${REPO}/python_compat:${ROS_INSTALL}/lib/python${pyver}/site-packages:${ROS_INSTALL}/local/lib/python${pyver}/dist-packages:${ROS_INSTALL}/lib/python3/dist-packages:${ROS_INSTALL}/lib/python${pyver}/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
  export ROS_PYTHON_VERSION=3
  export PYTHONNOUSERSITE=1
  # Boost 只用隔离 ROS 自带的一份（系统未装 libboost-dev）
  export BOOST_ROOT="${ROS_INSTALL}"
  export Boost_NO_SYSTEM_PATHS=ON
}

# -----------------------------------------------------------------------------
# deps：检查并安装系统依赖（headless 只需编译器；libsdl2-dev 仅供 GUI）
# -----------------------------------------------------------------------------
deps() {
  local pkgs=(
    build-essential      # g++ / make
    cmake
    libsdl2-dev          # 2D GUI（SDL2）；缺失时自动仅编 headless，不算错误
  )
  local miss=() p
  for p in "${pkgs[@]}"; do
    dpkg -s "${p}" >/dev/null 2>&1 || miss+=("${p}")
  done
  if [[ ${#miss[@]} -eq 0 ]]; then
    log "系统依赖已全部安装，无需 sudo。"
    return 0
  fi
  echo "需要 sudo 安装以下系统包（tool 工具允许使用 apt，见本工具 README.md）："
  printf '  %s\n' "${miss[@]}"
  sudo apt-get update
  sudo apt-get install -y "${miss[@]}"
  log "系统依赖安装完成。"
}

# -----------------------------------------------------------------------------
# build：CMake 配置 → 编译 → 安装
# -----------------------------------------------------------------------------
do_build() {
  [[ -d "${ROS_INSTALL}" ]] || fail "缺少隔离 ROS（${ROS_INSTALL}）。请先在仓库根目录执行 ./custom_mini.sh build"
  if ! rm -rf "${BDIR}" 2>/dev/null; then
    fail "无法清理 ${BDIR}（可能因曾用 sudo 构建导致 root 属主）。请先执行：\n  sudo chown -R \"\$USER:\$USER\" \"${BUILD}\"\n然后重新 ./build.sh（不要用 sudo）"
  fi
  mkdir -p "${BUILD}" "${PREFIX}"
  prepare_env

  log "CMake 配置 -> ${BDIR}（日志：${LOG}）"
  cmake -S "${TOOL}" -B "${BDIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DFLAT_SIM_ENABLE_GUI=ON \
    -DCMAKE_INSTALL_RPATH="${ROS_INSTALL}/lib" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON 2>&1 | tee "${LOG}"

  log "编译（-j${JOBS}）"
  cmake --build "${BDIR}" -- -j "${JOBS}" 2>&1 | tee -a "${LOG}"

  log "安装 -> ${PREFIX}"
  cmake --install "${BDIR}" 2>&1 | tee -a "${LOG}"

  log "完成：${PREFIX}/bin/flat_sim_node"
  log "运行：./tool/flat_sim/run_flat_sim.sh（或 --headless）"
}

clean() {
  rm -rf "${BUILD}" "${PREFIX}"
  log "已删除 build/ 与 install/（源码 src/、worlds/、models/ 保留）"
}

help() {
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

cmd="${1:-build}"
case "${cmd}" in
  deps)  deps ;;
  build) do_build ;;
  clean) clean ;;
  help|-h|--help) help ;;
  *) echo "未知子命令：${cmd}" >&2; help >&2; exit 1 ;;
esac

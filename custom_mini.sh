#!/usr/bin/env bash
# =============================================================================
# custom_mini.sh
# -----------------------------------------------------------------------------
# 定制最小环境：不依赖 ROS 的 package.xml / nodelet_plugins.xml。
# 插件列表写在 plugins.json，由 custom_mini_manager 用 class_loader 按 .so 路径加载。
#
# 子命令（无参数 = help）：
#   ./custom_mini.sh build     编译 → custom_mini_build/ + custom_mini_install/
#   ./custom_mini.sh package   打包 → custom_mini_runtime/（README/run/json 为软链接）
#   ./custom_mini.sh run       执行 custom_mini_runtime/run.sh
#   ./custom_mini.sh clean     删除编译中间文件 custom_mini_build/
#   ./custom_mini.sh help      参数说明
#
# 前提：先完成 ./official_full.sh build，本脚本只编 manager，库从 official_full_install 借。
# 日志：custom_mini_logs/build.log 以及 custom_mini_logs/01_cmake.log ...
# =============================================================================
set -euo pipefail

# -----------------------------------------------------------------------------
# 路径与常量（全部相对本脚本所在仓库根目录）
# -----------------------------------------------------------------------------
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${ROOT}/src"
DOC="${ROOT}/doc/custom_mini"                 # runtime 里 README / run.sh / plugins.json 的源
FULL_INSTALL="${ROOT}/official_full_install"  # 官方全量产物（优先使用）
MINI="${ROOT}/mini_install"                   # 旧前缀，仅作后备
PREFIX=""                                     # pick_prefix 之后才赋值
BUILD_DIR="${ROOT}/custom_mini_build"
INSTALL_DIR="${ROOT}/custom_mini_install"
RUNTIME="${ROOT}/custom_mini_runtime"
LOG_DIR="${ROOT}/custom_mini_logs"
PY_COMPAT="${ROOT}/python_compat"
MASTER_LOG=""                                 # prepare_log_dir 之后指向 build.log

# nlohmann/json 单头文件：manager 解析 plugins.json 用，不走系统包
JSON_DIR="${SRC}/nlohmann_json/include/nlohmann"
JSON_HPP="${JSON_DIR}/json.hpp"
JSON_URL="https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp"

JOBS="$(nproc 2>/dev/null || echo 4)"
PYVER="$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0], sys.version_info[1]))')"

# -----------------------------------------------------------------------------
# 帮助
# -----------------------------------------------------------------------------
cmd_help() {
  cat <<EOF
用法: $0 <build|package|run|clean|help>

  build     编译 custom_mini_manager
            中间文件: ${BUILD_DIR}
            安装目录: ${INSTALL_DIR}
            日志: ${LOG_DIR}
            前提: 已有 official_full_install/（./official_full.sh build）

  package   从 official_full_install 产物生成可拷贝的运行目录 ${RUNTIME}
            README.md / run.sh / plugins.json 是指向 ${DOC} 的软链接
            前提: 已执行 build

  run       执行 ${RUNTIME}/run.sh
            前提: 已执行 package

  clean     删除编译中间目录 ${BUILD_DIR}
            不删除 ${INSTALL_DIR} 和 ${RUNTIME}

  help      显示本说明

不带参数时执行 help。
EOF
}

# -----------------------------------------------------------------------------
# 日志：同时打到终端和 custom_mini_logs/build.log
# -----------------------------------------------------------------------------
log() {
  local msg="[$(date '+%F %T')] $*"
  echo "${msg}"
  if [[ -n "${MASTER_LOG}" ]]; then
    echo "${msg}" >> "${MASTER_LOG}"
  fi
}

# 出错立即停。已开始记总日志时，把原因也写进 build.log。
fail() {
  if [[ -n "${MASTER_LOG}" ]]; then
    log "ERROR: $*"
    log "Build stopped. See logs in ${LOG_DIR}"
  else
    echo "ERROR: $*" >&2
  fi
  exit 1
}

# 跑一步并把 stdout/stderr 写入 logs/<name>.log。
# 函数里临时 set +e：否则 set -e 会在命令失败时直接杀进程，拿不到退出码。
run_logged() {
  local name="$1"
  shift
  local logfile="${LOG_DIR}/${name}.log"
  local rc
  log "START ${name}"
  log "进度写入 ${logfile}"
  set +e
  "$@" >> "${logfile}" 2>&1
  rc=$?
  set -e
  if [[ "${rc}" -ne 0 ]]; then
    log "FAILED ${name} (see ${logfile})"
    tail -n 80 "${logfile}" || true
    fail "step ${name} failed"
  fi
  log "DONE  ${name}"
}

# 创建日志目录，并把后续 log() 同步到 build.log。
prepare_log_dir() {
  mkdir -p "${LOG_DIR}"
  MASTER_LOG="${LOG_DIR}/build.log"
  touch "${MASTER_LOG}"
}

# -----------------------------------------------------------------------------
# 依赖探测
# -----------------------------------------------------------------------------
# 选定 CMAKE_PREFIX_PATH：优先 official_full_install，其次旧的 mini_install。
# 判断条件：有 ROS 头文件 + libnodeletlib.so（说明全量已经编过）。
pick_prefix() {
  if [[ -d "${FULL_INSTALL}/include/ros" && -f "${FULL_INSTALL}/lib/libnodeletlib.so" ]]; then
    PREFIX="${FULL_INSTALL}"
  elif [[ -d "${MINI}/include/ros" && -f "${MINI}/lib/libnodeletlib.so" ]]; then
    PREFIX="${MINI}"
  else
    fail "需要先有 official_full_install/（请先 ./official_full.sh build）"
  fi
}

# manager 编译需要 nlohmann/json.hpp。已有则跳过，没有就下单头文件。
ensure_nlohmann_json() {
  if [[ -f "${JSON_HPP}" ]]; then
    return 0
  fi
  echo "下载 nlohmann/json → ${JSON_HPP}"
  mkdir -p "${JSON_DIR}"
  curl -fsSL --retry 2 --connect-timeout 10 --max-time 60 -o "${JSON_HPP}" "${JSON_URL}" \
    || fail "下载 nlohmann/json 失败"
}

# -----------------------------------------------------------------------------
# package 用的拷贝辅助
# -----------------------------------------------------------------------------
# 系统库不打进 runtime（目标机自己有 libc / libstdc++）。
is_system_lib() {
  local p="$1"
  case "${p}" in
    linux-vdso.so*|ld-linux*|ld-linux-x86-64.so*) return 0 ;;
    /lib/*|/lib64/*|/usr/lib/*|/usr/lib64/*) return 0 ;;
  esac
  return 1
}

# 用 ldd 列出 ELF 的非系统依赖路径。
#   "libfoo.so => /path/libfoo.so (0x...)"  → 取箭头后的绝对路径
#   有的行本身就是 "/path/libfoo.so (0x...)"
collect_nonsystem_libs() {
  local elf="$1"
  local line path
  ldd "${elf}" 2>/dev/null | while read -r line; do
    path=""
    if [[ "${line}" == *" => /"* ]]; then
      path="${line#* => }"
      path="${path%% (*}"
    elif [[ "${line}" == /* ]]; then
      path="${line%% (*}"
    fi
    [[ -n "${path}" ]] || continue
    is_system_lib "${path}" && continue
    printf '%s\n' "${path}"
  done
}

# 从 PREFIX 的几处 Python 安装位置里找包，拷到 dest。
# catkin 常用 dist-packages，pip 常用 site-packages。
copy_python_pkg() {
  local name="$1"
  local dest="$2"
  local src
  for src in \
    "${PREFIX}/lib/python3/dist-packages/${name}" \
    "${PREFIX}/lib/python${PYVER}/site-packages/${name}" \
    "${PREFIX}/local/lib/python${PYVER}/dist-packages/${name}"
  do
    if [[ -d "${src}" ]]; then
      cp -a "${src}" "${dest}/"
      return 0
    fi
    if [[ -f "${src}" ]]; then
      cp -a "${src}" "${dest}/"
      return 0
    fi
  done
  return 1
}

# 拷一份 .so：先跟 symlink 找到真实文件，再按需要补上「短名 → 带版本名」的链接。
# 例如 libtinyxml2.so.8 → libtinyxml2.so.8.0.0，运行时按 SONAME 才能找到。
copy_lib() {
  local lib="$1"
  local dest="$2"
  local real base realbase
  [[ -e "${lib}" ]] || return 0
  real="$(readlink -f "${lib}")"
  [[ -f "${real}" ]] || return 0
  cp -a "${real}" "${dest}/"
  base="$(basename "${lib}")"
  realbase="$(basename "${real}")"
  if [[ "${base}" != "${realbase}" ]]; then
    ln -sfn "${realbase}" "${dest}/${base}"
  fi
}

# runtime 里的 README / run.sh / plugins.json 不复制，只链到 doc/custom_mini/。
# 相对路径必须从 custom_mini_runtime/ 出发，所以是 ../doc/custom_mini/...
link_doc() {
  local name="$1"
  local src="${DOC}/${name}"
  [[ -e "${src}" ]] || fail "缺少 ${src}"
  ln -sfn "../doc/custom_mini/${name}" "${RUNTIME}/${name}"
}

# -----------------------------------------------------------------------------
# build：只编 custom_mini_manager
# -----------------------------------------------------------------------------
# CMAKE_PREFIX_PATH 指向官方 install，才能找到 roscpp / nodelet / class_loader。
# RPATH=$ORIGIN/../lib：装到 install/bin 后，运行时在同级 ../lib 找 .so，不靠 LD_LIBRARY_PATH。
cmd_build() {
  prepare_log_dir
  pick_prefix
  ensure_nlohmann_json
  [[ -f "${SRC}/custom_mini/src/custom_mini_manager.cpp" ]] || fail "缺少 src/custom_mini"
  [[ -f "${PREFIX}/lib/libmy_nodelet_plugin.so" ]] || fail "缺少 libmy_nodelet_plugin.so，请先编译 my_nodelet_plugin"

  log "==== custom_mini build ===="
  log "prefix=${PREFIX}"
  log "build=${BUILD_DIR}"
  log "install=${INSTALL_DIR}"
  log "logs=${LOG_DIR}"

  mkdir -p "${BUILD_DIR}" "${INSTALL_DIR}"
  run_logged 01_cmake cmake -S "${SRC}/custom_mini" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${PREFIX}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCMAKE_INSTALL_RPATH="\$ORIGIN/../lib" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DCMAKE_CXX_STANDARD=14
  run_logged 02_build cmake --build "${BUILD_DIR}" -- -j "${JOBS}"
  run_logged 03_install cmake --install "${BUILD_DIR}"

  [[ -x "${INSTALL_DIR}/bin/custom_mini_manager" ]] || fail "安装失败：没有 ${INSTALL_DIR}/bin/custom_mini_manager"
  log "==== BUILD SUCCESS ===="
  log "下一步: $0 package"
}

# -----------------------------------------------------------------------------
# package：抽出可拷走的 custom_mini_runtime/
# -----------------------------------------------------------------------------
# 内容：
#   bin/   manager + rosmaster（shebang 改成 /usr/bin/env python3，不指向 install）
#   lib/   插件 .so + ldd 扫到的非系统库
#   python/  rosmaster 需要的模块 + python_compat
#   软链接  README.md / run.sh / plugins.json → doc/custom_mini/
# 没有 share/：manager 不走 pluginlib / rospack。
cmd_package() {
  pick_prefix
  local manager="${INSTALL_DIR}/bin/custom_mini_manager"
  local plugin="${PREFIX}/lib/libmy_nodelet_plugin.so"
  [[ -x "${manager}" ]] || fail "缺少 ${manager}，请先: $0 build"
  [[ -f "${plugin}" ]] || fail "缺少 ${plugin}"
  [[ -x "${PREFIX}/bin/rosmaster" ]] || fail "缺少 ${PREFIX}/bin/rosmaster"
  [[ -d "${DOC}" ]] || fail "缺少 ${DOC}"

  echo "==== package → ${RUNTIME} ===="
  rm -rf "${RUNTIME}"
  mkdir -p "${RUNTIME}/bin" "${RUNTIME}/lib" "${RUNTIME}/python"

  cp -a "${manager}" "${RUNTIME}/bin/custom_mini_manager"
  cp -a "${plugin}" "${RUNTIME}/lib/"

  # manager 和插件各自 ldd，去重后拷进 runtime/lib
  local lib
  while read -r lib; do
    copy_lib "${lib}" "${RUNTIME}/lib"
  done < <(
    {
      collect_nonsystem_libs "${manager}"
      collect_nonsystem_libs "${plugin}"
    } | sort -u
  )

  # install 里的 shebang 常指向 official_full_install/bin/python3，拷走后会失效
  sed '1s|^#!.*|#!/usr/bin/env python3|' "${PREFIX}/bin/rosmaster" > "${RUNTIME}/bin/rosmaster"
  chmod +x "${RUNTIME}/bin/rosmaster"

  copy_python_pkg rosmaster "${RUNTIME}/python" || fail "缺少 python 包 rosmaster"
  copy_python_pkg rosgraph "${RUNTIME}/python" || fail "缺少 python 包 rosgraph"
  copy_python_pkg rospkg "${RUNTIME}/python" || fail "缺少 python 包 rospkg"
  copy_python_pkg defusedxml "${RUNTIME}/python" || fail "缺少 python 包 defusedxml"
  copy_python_pkg yaml "${RUNTIME}/python" || true
  copy_python_pkg catkin_pkg "${RUNTIME}/python" || true
  copy_python_pkg distro "${RUNTIME}/python" || true
  cp -a "${PY_COMPAT}/sitecustomize.py" "${PY_COMPAT}/imp.py" "${RUNTIME}/python/"
  find "${RUNTIME}/python" -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true

  link_doc README.md
  link_doc run.sh
  link_doc plugins.json
  chmod +x "${DOC}/run.sh"

  echo "==== PACKAGE SUCCESS ===="
  echo "runtime: ${RUNTIME}"
  echo "下一步: $0 run"
}

# exec：用 run.sh 替换当前进程，Ctrl+C 由 run.sh 自己处理。
cmd_run() {
  [[ -e "${RUNTIME}/run.sh" ]] || fail "未找到 ${RUNTIME}/run.sh，请先: $0 package"
  exec "${RUNTIME}/run.sh"
}

# 只删编译中间文件。install / runtime / logs 都留着，避免误清可运行产物。
cmd_clean() {
  echo "删除 ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
  echo "clean 完成（保留 ${INSTALL_DIR} 与 ${RUNTIME}）"
}

# -----------------------------------------------------------------------------
# 入口
# -----------------------------------------------------------------------------
case "${1:-help}" in
  build) cmd_build ;;
  package) cmd_package ;;
  run) cmd_run ;;
  clean) cmd_clean ;;
  help|-h|--help) cmd_help ;;
  *) echo "未知命令: $1" >&2; cmd_help; exit 1 ;;
esac

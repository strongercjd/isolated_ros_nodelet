#!/usr/bin/env bash
# =============================================================================
# custom_mini.sh
# -----------------------------------------------------------------------------
# 定制最小 ROS 环境（不含应用层）。应用层在 app/，各包用自己的 make.sh。
#
# 子命令（无参数 = help）：
#   ./custom_mini.sh build     编译 ROS 栈 → custom_mini_build/ + custom_mini_install/
#   ./custom_mini.sh package   打包 → custom_mini_runtime/（README/run/env 为软链接）
#   ./custom_mini.sh run       打开 ROS 运行环境（交互式 shell）
#   ./custom_mini.sh clean     删除编译中间文件 custom_mini_build/
#   ./custom_mini.sh help      参数说明
#
# 独立：自己编一整套 ROS 栈到 custom_mini_install/。
# 不读取 official_full_install/（即使对方已编过也会再编进本目录）。
# 日志：custom_mini_logs/build.log 以及 01_python.log … 09_tree.log
# =============================================================================
set -euo pipefail

# -----------------------------------------------------------------------------
# 路径与常量（全部相对本脚本所在仓库根目录）
# -----------------------------------------------------------------------------
# 仓库根目录（脚本所在目录），后面所有路径都相对它。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${ROOT}/src"
INSTALL="${ROOT}/custom_mini_install"
DOC="${ROOT}/doc/custom_mini"                 # runtime 里 README / run.sh / env.sh 的源
RUNTIME="${ROOT}/custom_mini_runtime"
LOG_DIR="${ROOT}/custom_mini_logs"
BUILD_DIR="${ROOT}/custom_mini_build"
BUILD_TP="${BUILD_DIR}/third_party"             # Boost / Poco 等非 catkin 的中间文件
BUILD_ISOLATED="${BUILD_DIR}/isolated"          # catkin_make_isolated 的 --build-space
DEVEL_ISOLATED="${BUILD_DIR}/devel"             # catkin_make_isolated 的 --devel-space
PY_COMPAT="${ROOT}/python_compat"
# pip --target 写死 3.12：当前按 Ubuntu 24.04 验证。换发行版要改这一处。
PY_SITE="${INSTALL}/lib/python3.12/site-packages"
JOBS="$(nproc 2>/dev/null || echo 4)"
PYVER="$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0], sys.version_info[1]))')"
MASTER_LOG=""                                   # prepare_build_env 之后指向 build.log

# -----------------------------------------------------------------------------
# 帮助
# -----------------------------------------------------------------------------
cmd_help() {
  cat <<EOF
用法: $0 <build|package|run|clean|help>

  build     独立编译隔离 ROS 栈（不含 app/ 应用层）
            中间文件: ${BUILD_DIR}
            安装目录: ${INSTALL}
            日志: ${LOG_DIR}
            不使用 official_full_install/

  package   从 ${INSTALL} 抽出可拷贝的 ROS 运行目录 ${RUNTIME}
            README.md / run.sh / env.sh 指向 ${DOC}
            前提: 已执行 build

  run       执行 ${RUNTIME}/run.sh（打开已注入环境的交互式 shell）
            前提: 已执行 package
            应用层 demo 请用 app/*/make.sh 后执行 app_runtime/run.sh

  clean     删除编译中间目录 ${BUILD_DIR}
            不删除 ${INSTALL} 和 ${RUNTIME}

  help      显示本说明

不带参数时执行 help。
EOF
}

# -----------------------------------------------------------------------------
# 日志与失败处理
# -----------------------------------------------------------------------------
# 同时打到终端和 logs/build.log，便于对照分步日志。
log() {
  local msg="[$(date '+%F %T')] $*"
  echo "${msg}"
  if [[ -n "${MASTER_LOG}" ]]; then
    echo "${msg}" >> "${MASTER_LOG}"
  fi
}

# 出错立即停。调用方不要捕获后继续。
fail() {
  if [[ -n "${MASTER_LOG}" ]]; then
    log "ERROR: $*"
    log "Build stopped. See logs in ${LOG_DIR}"
  else
    echo "ERROR: $*" >&2
  fi
  exit 1
}

# git 检出常把脚本存成 100644。给带真实 shebang 的文件补 +x。
# cmake configure_file 会把模板权限拷到生成的 env.sh 等。
# 跳过 #!@...（如 _setup_util.py.in / script.py.in）：上游就是 644，
# 安装态由 catkin_install_python(PROGRAMS) 再设可执行位。
chmod_shebang_files() {
  local f head
  [[ $# -gt 0 ]] || return 0
  while IFS= read -r -d '' f; do
    head="$(head -c 3 "${f}" 2>/dev/null || true)"
    [[ "${head}" == '#!@' ]] && continue
    if [[ "${head:0:2}" == '#!' ]]; then
      chmod +x "${f}" 2>/dev/null || true
    fi
  done < <(find "$@" -type f -print0 2>/dev/null)
}

# 跑一个步骤：stdout/stderr 进 logs/<name>.log；失败时把末尾 80 行打到终端。
# 注意：函数体内临时 set +e，才能拿到真实退出码（否则 set -e 会直接杀进程）。
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

# -----------------------------------------------------------------------------
# 编译前环境：目录、红线、工具、隔离变量
# -----------------------------------------------------------------------------
# CMAKE_PREFIX_PATH / BOOST_ROOT 只指向本仓库 install，避免 cmake 捡到系统 ROS / Boost。
# PYTHONNOUSERSITE=1：不用 ~/.local 里的包。
# install/bin/python → 系统 python3：ROS 脚本 shebang 常写 #!/usr/bin/env python。
prepare_build_env() {
  mkdir -p "${SRC}" "${INSTALL}" "${LOG_DIR}" "${BUILD_TP}" "${PY_SITE}" \
    "${INSTALL}/bin" "${INSTALL}/lib" "${INSTALL}/include"
  MASTER_LOG="${LOG_DIR}/build.log"
  touch "${MASTER_LOG}"

  case "${INSTALL}" in
    /usr|/usr/*|/opt|/opt/*|/usr/local|/usr/local/*)
      fail "refusing to install into system path: ${INSTALL}"
      ;;
  esac

  command -v cmake >/dev/null || fail "cmake is required"
  command -v g++ >/dev/null || fail "g++ is required"
  command -v make >/dev/null || fail "make is required"
  command -v python3 >/dev/null || fail "python3 is required"
  command -v curl >/dev/null || fail "curl is required"
  command -v tar >/dev/null || fail "tar is required"

  # Never use system ROS / system Boost headers.
  export CMAKE_PREFIX_PATH="${INSTALL}"
  export PKG_CONFIG_PATH="${INSTALL}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
  export LD_LIBRARY_PATH="${INSTALL}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="${INSTALL}/bin:${INSTALL}/local/bin:${PATH}"
  export PYTHONPATH="${PY_COMPAT}:${PY_SITE}:${INSTALL}/local/lib/python3.12/dist-packages:${INSTALL}/lib/python3/dist-packages:${INSTALL}/lib/python3.12/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
  export BOOST_ROOT="${INSTALL}"
  export Boost_NO_SYSTEM_PATHS=ON
  export ROS_PYTHON_VERSION=3
  export ROS_DISTRO=noetic
  export PYTHONNOUSERSITE=1

  # Provide `python` for ROS shebangs.
  if [[ ! -e "${INSTALL}/bin/python" ]]; then
    ln -sfn "$(command -v python3)" "${INSTALL}/bin/python"
  fi
  if [[ ! -e "${INSTALL}/bin/python3" ]]; then
    ln -sfn "$(command -v python3)" "${INSTALL}/bin/python3"
  fi
}

# -----------------------------------------------------------------------------
# 下载与源码
# -----------------------------------------------------------------------------
curl_get() {
  # 短超时 + 少次重试，避免不存在的 GitHub 分支把构建卡住数分钟。
  local url="$1"
  local out="$2"
  curl -fsSL --retry 2 --retry-delay 1 --connect-timeout 10 --max-time 90 \
    -o "${out}" "${url}"
}

# 拉取 GitHub 仓库快照到 dest。先试 heads/<branch>，再试 tags/<branch>。
# 目录非空则跳过（支持断点续编，也支持用户预先 git clone）。
# 用法：fetch_github <org> <repo> <dest> <branch-or-tag> [备选 ...]
fetch_github() {
  local org="$1"
  local repo="$2"
  local dest="$3"
  shift 3
  if [[ -d "${dest}" && -n "$(ls -A "${dest}" 2>/dev/null)" ]]; then
    log "skip fetch ${org}/${repo} (already at ${dest})"
    return 0
  fi
  local br url tmp
  for br in "$@"; do
    tmp="${LOG_DIR}/${repo}-${br}.tar.gz"
    # codeload.github.com 优先（某些网络下 github.com 连接超时，codeload 可达），再退回 archive 路径。
    for url in \
      "https://codeload.github.com/${org}/${repo}/tar.gz/refs/heads/${br}" \
      "https://codeload.github.com/${org}/${repo}/tar.gz/refs/tags/${br}" \
      "https://github.com/${org}/${repo}/archive/refs/heads/${br}.tar.gz" \
      "https://github.com/${org}/${repo}/archive/refs/tags/${br}.tar.gz"
    do
      log "try ${url}"
      if curl_get "${url}" "${tmp}"; then
        mkdir -p "${dest}"
        tar -xzf "${tmp}" -C "${dest}" --strip-components=1
        log "fetched ${org}/${repo} branch/tag ${br}"
        return 0
      fi
    done
  done
  fail "unable to fetch ${org}/${repo} (tried: $*)"
}

# 给指定 catkin 包目录放 CATKIN_IGNORE，跳过 rosbag 等 demo 不需要的包。
# 在 src/ 里找 <name>...</name> 的 package.xml（跳过 boost/poco 等非 ROS 树）。
# catkin 见到 CATKIN_IGNORE 就不会编这个包。
ignore_package() {
  local name="$1"
  local pkgxml
  pkgxml="$(find "${SRC}" \
    \( -path "${SRC}/boost" -o -path "${SRC}/poco" -o -path "${SRC}/tinyxml2" \
       -o -path "${SRC}/console_bridge" -o -path "${SRC}/libuuid" \) -prune \
    -o -name package.xml -print 2>/dev/null \
    | while read -r f; do
        grep -q "<name>${name}</name>" "${f}" && echo "${f}" && break
      done)"
  if [[ -n "${pkgxml}" ]]; then
    touch "$(dirname "${pkgxml}")/CATKIN_IGNORE"
    log "CATKIN_IGNORE ${name} ($(dirname "${pkgxml}"))"
  fi
}

# -----------------------------------------------------------------------------
# 普通 CMake 第三方：配置、编译、安装到 install/
# -----------------------------------------------------------------------------
# stamp 文件 ${INSTALL}/.stamp_<name> 表示这一步已经成功过，再跑 build 会跳过。
# RPATH 写成 install/lib，编出来的 .so 彼此能找到，不靠系统路径。
# 额外的 cmake 参数通过 "$@" 传进来（例如 -DBUILD_TESTING=OFF）。
cmake_build_install() {
  local name="$1"
  local srcdir="$2"
  shift 2
  if [[ -f "${INSTALL}/.stamp_${name}" ]]; then
    log "skip ${name} (already installed)"
    return 0
  fi
  local bdir="${BUILD_TP}/${name}"
  rm -rf "${bdir}"
  mkdir -p "${bdir}"
  (
    # run_logged 以 set +e 调用本函数，必须在这里 set -e，
    # 否则 cmake 失败也会继续 touch stamp，下次 build 会错误地 skip。
    set -e
    cd "${bdir}"
    cmake "${srcdir}" \
      -DCMAKE_INSTALL_PREFIX="${INSTALL}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="${INSTALL}" \
      -DCMAKE_INSTALL_RPATH="${INSTALL}/lib" \
      -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
      -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DBUILD_SHARED_LIBS=ON \
      "$@"
    cmake --build . -- -j "${JOBS}"
    cmake --install .
  )
  touch "${INSTALL}/.stamp_${name}"
}

# ---------------------------------------------------------------------------
# 1) Python：pip 装到 install/，不碰系统 site-packages
# ---------------------------------------------------------------------------
# sitecustomize.py / imp.py 拷进 PY_SITE：解释器启动会自动加载（PYTHONPATH 已含 PY_SITE）。
# empy 必须 3.3.4：4.x API 变了，catkin 用不了。
# setuptools 69 自带 distutils，给 Python 3.12 上的 rosclean / roslaunch 用。
setup_python() {
  mkdir -p "${PY_SITE}"
  cp -f "${PY_COMPAT}/sitecustomize.py" "${PY_SITE}/sitecustomize.py"
  cp -f "${PY_COMPAT}/imp.py" "${PY_SITE}/imp.py"
  export PIP_BREAK_SYSTEM_PACKAGES=1
  # 以本 install 前缀为准，避免 official_full 已装过导致本目录跳过。
  if [[ -d "${PY_SITE}/catkin_pkg" ]] && PYTHONPATH="${PY_COMPAT}:${PY_SITE}" python3 -c "import catkin_pkg, em, yaml, rospkg" >/dev/null 2>&1; then
    log "skip python pip deps (already in ${PY_SITE})"
  else
    local getpip="${LOG_DIR}/get-pip.py"
    if ! python3 -c "import pip" >/dev/null 2>&1; then
      log "bootstrap pip into ${INSTALL}"
      if [[ ! -s "${getpip}" || "$(wc -c < "${getpip}")" -lt 100000 ]]; then
        rm -f "${getpip}"
        curl_get "https://bootstrap.pypa.io/get-pip.py" "${getpip}"
      fi
      python3 "${getpip}" --prefix="${INSTALL}" --no-warn-script-location
    fi
    python3 -m pip install --target="${PY_SITE}" --upgrade \
      "setuptools==69.5.1" \
      "wheel" \
      "packaging" \
      "catkin_pkg" \
      "empy==3.3.4" \
      "PyYAML" \
      "rospkg" \
      "rosdistro" \
      "defusedxml" \
      "distro" \
      "python-dateutil" \
      "docutils"
    python3 -c "import catkin_pkg, em, yaml, rospkg" \
      || fail "python deps still missing after pip install"
  fi
  if [[ -d "${PY_SITE}/bin" ]]; then
    mkdir -p "${INSTALL}/bin"
    cp -f "${PY_SITE}/bin/"* "${INSTALL}/bin/" 2>/dev/null || true
    chmod +x "${INSTALL}/bin/"* 2>/dev/null || true
  fi

  # Headers matching the system libpython (extracted from distro .deb, not apt-installed).
  # 只 download + dpkg-deb -x，不 apt install，避免往系统写开发包。
  if [[ ! -f "${INSTALL}/include/x86_64-linux-gnu/python${PYVER}/pyconfig.h" && ! -f "${INSTALL}/include/python${PYVER}/Python.h" ]]; then
    local debdir="${SRC}/python_dev"
    mkdir -p "${debdir}"
    log "extract libpython${PYVER}-dev headers into install/"
    (
      cd "${debdir}"
      rm -rf extracted
      apt-get download "libpython${PYVER}-dev"
      dpkg-deb -x libpython${PYVER}-dev_*.deb extracted
    )
    mkdir -p "${INSTALL}/include"
    cp -a "${debdir}/extracted/usr/include/." "${INSTALL}/include/"
  fi
  # rospy / 部分扩展要链 libpython。用系统 .so.1.0，不编 Python 本身。
  if [[ ! -e "${INSTALL}/lib/libpython${PYVER}.so" ]]; then
    local pylib
    pylib="$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))')/libpython${PYVER}.so.1.0"
    [[ -f "${pylib}" ]] || fail "system libpython${PYVER}.so.1.0 not found"
    ln -sfn "${pylib}" "${INSTALL}/lib/libpython${PYVER}.so"
  fi
}

# Eigen 纯头文件库（app/slam2d_nodelet 的 ICP SVD 用）。与 python-dev 同套路：
# 只 download + dpkg-deb -x，不 apt install，避免往系统写开发包。
setup_eigen() {
  if [[ -f "${INSTALL}/include/eigen3/Eigen/Core" ]]; then
    log "skip eigen headers (already at ${INSTALL}/include/eigen3)"
    return 0
  fi
  local debdir="${SRC}/eigen3_dev"
  mkdir -p "${debdir}"
  log "extract libeigen3-dev headers into install/"
  (
    cd "${debdir}"
    rm -rf extracted libeigen3-dev_*.deb
    apt-get download libeigen3-dev
    dpkg-deb -x libeigen3-dev_*.deb extracted
  )
  cp -a "${debdir}/extracted/usr/include/eigen3" "${INSTALL}/include/eigen3"
  mkdir -p "${INSTALL}/share"
  cp -a "${debdir}/extracted/usr/share/eigen3/cmake" "${INSTALL}/share/eigen3/cmake" 2>/dev/null || true
  [[ -f "${INSTALL}/include/eigen3/Eigen/Core" ]] || fail "eigen headers missing after extract"
}

# ---------------------------------------------------------------------------
# 2) 第三方 C/C++ 库（源码 -> install/）
# ---------------------------------------------------------------------------
# libuuid：本仓库最小实现，给 bondcpp 用，避免系统 uuid-dev。
build_uuid() {
  cmake_build_install libuuid "${SRC}/libuuid"
}

# pluginlib / rospack 解析 XML。
build_tinyxml2() {
  fetch_github leethomason tinyxml2 "${SRC}/tinyxml2" 8.0.0
  cmake_build_install tinyxml2 "${SRC}/tinyxml2" -Dtinyxml2_BUILD_TESTING=OFF
}

# ROS 与底层日志桥。源码树里也有 package.xml，打 CATKIN_IGNORE 以免 catkin 再编一遍。
build_console_bridge() {
  fetch_github ros console_bridge "${SRC}/console_bridge" 1.0.2 noetic-devel master
  cmake_build_install console_bridge "${SRC}/console_bridge" -DBUILD_TESTING=OFF
  touch "${SRC}/console_bridge/CATKIN_IGNORE" 2>/dev/null || true
}

# class_loader 只用 Poco Foundation 做 dlopen。其余模块全部关掉，缩短编译、少依赖。
build_poco() {
  fetch_github pocoproject poco "${SRC}/poco" poco-1.11.8-release
  cmake_build_install poco "${SRC}/poco" \
    -DENABLE_XML=OFF \
    -DENABLE_JSON=OFF \
    -DENABLE_NET=OFF \
    -DENABLE_NETSSL=OFF \
    -DENABLE_CRYPTO=OFF \
    -DENABLE_DATA=OFF \
    -DENABLE_DATA_SQLITE=OFF \
    -DENABLE_DATA_MYSQL=OFF \
    -DENABLE_DATA_POSTGRESQL=OFF \
    -DENABLE_MONGODB=OFF \
    -DENABLE_REDIS=OFF \
    -DENABLE_JWT=OFF \
    -DENABLE_ZIP=OFF \
    -DENABLE_PAGECOMPILER=OFF \
    -DENABLE_PAGECOMPILER_FILE2PAGE=OFF \
    -DENABLE_ACTIVERECORD=OFF \
    -DENABLE_ACTIVERECORD_COMPILER=OFF \
    -DENABLE_ENCODINGS=OFF \
    -DENABLE_ENCODINGS_COMPILER=OFF \
    -DENABLE_UTIL=OFF \
    -DENABLE_SEVENZIP=OFF \
    -DENABLE_TESTS=OFF \
    -DBUILD_TESTING=OFF
}

# glibc 2.34+ 里 PTHREAD_STACK_MIN 不再是编译期常量，Boost 1.71 的 #if 会炸。
# 改成 #ifdef 即可，语义仍然是「定义了就用」。
patch_boost_171() {
  local hdr="${SRC}/boost/boost_1_71_0/boost/thread/pthread/thread_data.hpp"
  if [[ -f "${hdr}" ]] && grep -q '#if PTHREAD_STACK_MIN > 0' "${hdr}"; then
    # glibc 2.34+ makes PTHREAD_STACK_MIN a non-constant macro.
    sed -i 's/#if PTHREAD_STACK_MIN > 0/#ifdef PTHREAD_STACK_MIN/' "${hdr}"
    log "patched Boost 1.71 PTHREAD_STACK_MIN for modern glibc"
  fi
}

# Boost 1.71：roscpp / rospack / class_loader 需要。
# 跳过条件必须「库 + 头文件」都在：目录改名后曾出现库装到旧 prefix、头文件缺失。
# bootstrap 的 prefix 写在 project-config.jam 里，和当前 INSTALL 不一致就要重跑。
build_boost() {
  if [[ -f "${INSTALL}/lib/libboost_thread.so" && -d "${INSTALL}/include/boost" ]]; then
    log "skip Boost (already installed)"
    return 0
  fi
  mkdir -p "${SRC}/boost"
  local tarball="${SRC}/boost/boost_1_71_0.tar.gz"
  if [[ ! -f "${tarball}" ]]; then
    log "download Boost 1.71.0 from archives.boost.io (fallback: sourceforge)"
    if ! curl_get "https://archives.boost.io/release/1.71.0/source/boost_1_71_0.tar.gz" "${tarball}"; then
      curl_get "https://sourceforge.net/projects/boost/files/boost/1.71.0/boost_1_71_0.tar.gz/download" "${tarball}"
    fi
  fi
  if [[ ! -d "${SRC}/boost/boost_1_71_0" ]]; then
    tar -xzf "${tarball}" -C "${SRC}/boost"
  fi
  patch_boost_171
  (
    cd "${SRC}/boost/boost_1_71_0"
    chmod +x ./bootstrap.sh tools/build/src/engine/build.sh 2>/dev/null || true
    chmod +x ./b2 2>/dev/null || true
    # 目录改名后旧 project-config.jam 仍指向 install/，必须按当前 INSTALL 重跑 bootstrap
    if [[ ! -x ./b2 ]] || ! grep -q "option.set prefix : ${INSTALL} ;" project-config.jam 2>/dev/null; then
      ./bootstrap.sh --prefix="${INSTALL}" \
        --with-libraries=system,filesystem,thread,chrono,date_time,regex,atomic,program_options
    fi
    ./b2 --prefix="${INSTALL}" -j "${JOBS}" \
      variant=release \
      link=shared \
      threading=multi \
      cxxflags="-fPIC -std=c++14 -DBOOST_NO_CXX98_FUNCTION_BASE -fpermissive" \
      install
  )
}

# ---------------------------------------------------------------------------
# 3) 拉取 ROS Noetic 源码（GitHub，优先 noetic-devel）
# ---------------------------------------------------------------------------
# 每个 fetch_github 后面的名字是「先试这个分支/标签，失败再试下一个」。
# geneus / gennodejs 不在 github.com/ros 下，见 README 表格。
# 拉完后给 demo 用不到的包打 CATKIN_IGNORE（rosbag 会拖 lz4/bzip2）。
fetch_ros() {
  fetch_github ros catkin "${SRC}/catkin" noetic-devel
  fetch_github ros cmake_modules "${SRC}/cmake_modules" noetic-devel 0.5-devel kinetic-devel
  fetch_github ros genmsg "${SRC}/genmsg" noetic-devel kinetic-devel
  fetch_github ros gencpp "${SRC}/gencpp" noetic-devel kinetic-devel
  fetch_github ros genpy "${SRC}/genpy" noetic-devel kinetic-devel
  # geneus / gennodejs 不在 github.com/ros 下，见 README 表格。
  fetch_github jsk-ros-pkg geneus "${SRC}/geneus" master
  fetch_github RethinkRobotics-opensource gennodejs "${SRC}/gennodejs" kinetic-devel
  fetch_github ros genlisp "${SRC}/genlisp" noetic-devel kinetic-devel
  fetch_github ros message_generation "${SRC}/message_generation" noetic-devel kinetic-devel
  fetch_github ros message_runtime "${SRC}/message_runtime" noetic-devel kinetic-devel
  fetch_github ros std_msgs "${SRC}/std_msgs" noetic-devel kinetic-devel
  fetch_github ros ros_comm_msgs "${SRC}/ros_comm_msgs" noetic-devel kinetic-devel
  fetch_github ros roscpp_core "${SRC}/roscpp_core" noetic-devel kinetic-devel
  fetch_github ros rosconsole "${SRC}/rosconsole" noetic-devel kinetic-devel
  fetch_github ros ros "${SRC}/ros" noetic-devel
  fetch_github ros rospack "${SRC}/rospack" noetic-devel kinetic-devel
  fetch_github ros ros_comm "${SRC}/ros_comm" noetic-devel
  fetch_github ros pluginlib "${SRC}/pluginlib" noetic-devel 1.13-devel kinetic-devel
  fetch_github ros class_loader "${SRC}/class_loader" noetic-devel kinetic-devel
  fetch_github ros nodelet_core "${SRC}/nodelet_core" noetic-devel kinetic-devel indigo-devel
  fetch_github ros bond_core "${SRC}/bond_core" noetic-devel kinetic-devel
  # common_msgs：sensor_msgs / nav_msgs / geometry_msgs 等，应用层与 flat_sim 需要。
  fetch_github ros common_msgs "${SRC}/common_msgs" noetic-devel kinetic-devel

  # Packages not needed for the Talker/Listener demo (extra deps: lz4, bzip2, ...).
  ignore_package rosbag
  ignore_package rosbag_storage
  ignore_package roslz4
  ignore_package topic_tools
  ignore_package roswtf
  ignore_package nodelet_topic_tools
  ignore_package test_nodelet
  ignore_package test_nodelet_topic_tools
  ignore_package nodelet_tutorial_math
  ignore_package pluginlib_tutorials
  ignore_package test_bond
  ignore_package roscreate
  ignore_package rosmake
  ignore_package mk
  ignore_package rosbuild
  ignore_package rosboost_cfg
  ignore_package roscpp_tutorials
  ignore_package rospy_tutorials
  touch "${SRC}/ros_comm/test/CATKIN_IGNORE" 2>/dev/null || true
  touch "${SRC}/cmake_modules/tests/CATKIN_IGNORE" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# 4) catkin 隔离编译：ROS 核心 -> install/（不含 app/ 应用层）
# ---------------------------------------------------------------------------
# --source "${SRC}"：src/ 下带 package.xml 的目录都会被扫到。
# 应用层在 app/，不会被本步编进来（各包用自己的 make.sh）。
# ROSCONSOLE_BACKEND=print：不链 log4cxx，少一个系统依赖。
# Boost / UUID 全部指到本仓库 install，防止 FindBoost 捡到系统包。
build_ros() {
  chmod +x "${SRC}/catkin/bin/"* 2>/dev/null || true
  chmod_shebang_files "${SRC}/catkin/cmake/templates" "${SRC}/catkin/bin"
  # _setup_util.py.in 上游是 644（#!@ 不在上面统一 chmod）。但 configure_file 会把
  # 模板权限拷到 devel/_setup_util.py，而 setup.sh 要直接执行它；必须给模板 +x。
  chmod +x "${SRC}/catkin/cmake/templates/_setup_util.py.in" 2>/dev/null || true
  # 已生成的副本若仍是 644，cmake 可能先执行再覆盖。
  find "${DEVEL_ISOLATED}" "${INSTALL}" -name '_setup_util.py' -exec chmod +x {} + 2>/dev/null || true
  [[ -x "${SRC}/catkin/bin/catkin_make_isolated" ]] || fail "catkin_make_isolated missing"
  python3 -c "import catkin_pkg, em, yaml, rospkg" \
    || fail "python deps missing (catkin_pkg/empy/PyYAML/rospkg)"

  local extra_flags="-I${INSTALL}/include -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DBOOST_NO_CXX98_FUNCTION_BASE -Wno-deprecated-declarations -fpermissive"
  "${SRC}/catkin/bin/catkin_make_isolated" \
    --source "${SRC}" \
    --install \
    --install-space "${INSTALL}" \
    --devel-space "${DEVEL_ISOLATED}" \
    --build-space "${BUILD_ISOLATED}" \
    -j "${JOBS}" \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="${INSTALL}" \
      -DCMAKE_INSTALL_PREFIX="${INSTALL}" \
      -DCMAKE_INSTALL_RPATH="${INSTALL}/lib" \
      -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
      -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
      -DCMAKE_CXX_STANDARD=14 \
      -DCMAKE_CXX_STANDARD_REQUIRED=ON \
      "-DCMAKE_CXX_FLAGS=${extra_flags}" \
      -DPYTHON_EXECUTABLE="$(command -v python3)" \
      -DPYTHON_VERSION="${PYVER}" \
      -DPYTHON_INCLUDE_DIR="${INSTALL}/include/python${PYVER}" \
      -DPYTHON_LIBRARY="${INSTALL}/lib/libpython${PYVER}.so" \
      -DBoost_NO_SYSTEM_PATHS=ON \
      -DBoost_NO_BOOST_CMAKE=ON \
      -DBOOST_ROOT="${INSTALL}" \
      -DBoost_INCLUDE_DIR="${INSTALL}/include" \
      -DBoost_LIBRARY_DIR="${INSTALL}/lib" \
      -DBoost_USE_STATIC_LIBS=OFF \
      -DROSCONSOLE_BACKEND=print \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DBUILD_TESTING=OFF \
      -DCMAKE_POLICY_DEFAULT_CMP0148=OLD \
      -DUUID_INCLUDE_DIRS="${INSTALL}/include" \
      -DUUID_LIBRARIES="${INSTALL}/lib/libuuid.so"
}

# 编完后留一份目录清单，方便对照「到底装了什么」。跳过日志和中间文件以免撑爆。
write_final_tree() {
  (cd "${ROOT}" && find . \
    \( -path './.git' -o -path './custom_mini_logs' -o -path './custom_mini_build' \) -prune \
    -o -print) > "${LOG_DIR}/final_tree.txt"
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

# 从 install 的几处 Python 位置里找包。第二参数默认 runtime/python。
# 必须写 dest="${2:-...}"：set -u 下裸写 $2 且调用方没传时会直接失败。
copy_python_pkg() {
  local name="$1"
  local dest="${2:-${RUNTIME}/python}"
  local src
  for src in \
    "${INSTALL}/lib/python3/dist-packages/${name}" \
    "${INSTALL}/lib/python${PYVER}/site-packages/${name}" \
    "${INSTALL}/local/lib/python${PYVER}/dist-packages/${name}"
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

# 整包拷 share/<name>（package.xml、nodelet_plugins.xml、cmake 配置等）。
copy_share_pkg() {
  local name="$1"
  local src="${INSTALL}/share/${name}"
  [[ -d "${src}" ]] || return 1
  mkdir -p "${RUNTIME}/share"
  cp -a "${src}" "${RUNTIME}/share/"
}

# 拷 install/bin 下的入口。
#   无 shebang（ELF，如 rospack）→ 原样拷
#   #!...python...     → 改成 #!/usr/bin/env python3（去掉指向 install 的解释器）
#   #!...bash...       → 改成 #!/usr/bin/env bash（rosrun 是 bash，不能当 Python 跑）
# 先看前两个字节是不是 #!，避免对 ELF 做 head -n 1（二进制里有 NUL，bash 会报警）。
copy_bin_script() {
  local name="$1"
  local src="${INSTALL}/bin/${name}"
  local line
  [[ -e "${src}" ]] || fail "缺少 ${src}，请先成功完成: $0 build"
  if [[ "$(head -c 2 "${src}")" != "#!" ]]; then
    cp -a "${src}" "${RUNTIME}/bin/"
    return 0
  fi
  line="$(head -n 1 "${src}")"
  if [[ "${line}" == "#!"*python* ]]; then
    sed '1s|^#!.*|#!/usr/bin/env python3|' "${src}" > "${RUNTIME}/bin/${name}"
  elif [[ "${line}" == "#!"*bash* ]]; then
    sed '1s|^#!.*|#!/usr/bin/env bash|' "${src}" > "${RUNTIME}/bin/${name}"
  else
    cp -a "${src}" "${RUNTIME}/bin/"
    return 0
  fi
  chmod +x "${RUNTIME}/bin/${name}"
}

# catkin 约定：可执行文件在 lib/<pkg>/<name>，rosrun 靠这个路径找。
copy_pkg_bin() {
  local pkg="$1"
  local name="$2"
  local src="${INSTALL}/lib/${pkg}/${name}"
  [[ -x "${src}" ]] || fail "缺少 ${src}，请先成功完成: $0 build"
  mkdir -p "${RUNTIME}/lib/${pkg}"
  cp -a "${src}" "${RUNTIME}/lib/${pkg}/"
}

# runtime 里的 README / run.sh / env.sh 不复制，只链到 doc/custom_mini/。
link_doc() {
  local name="$1"
  local src="${DOC}/${name}"
  [[ -e "${src}" ]] || fail "缺少 ${src}"
  ln -sfn "../doc/custom_mini/${name}" "${RUNTIME}/${name}"
}

# -----------------------------------------------------------------------------
# build：只编 ROS 栈（应用层见 app/*/make.sh）
# -----------------------------------------------------------------------------
cmd_build() {
  # 避免误把官方路径留下的插件软链编进本环境
  for stale in my_nodelet_plugin talker_nodelet heartbeat_nodelet listener_nodelet; do
    if [[ -L "${SRC}/${stale}" ]]; then
      rm -f "${SRC}/${stale}"
      log "removed stale symlink ${SRC}/${stale}"
    fi
  done

  prepare_build_env
  log "==== custom_mini build（仅 ROS 环境，不使用 official_full_install）===="
  log "ROOT=${ROOT}  JOBS=${JOBS}"
  log "isolation: install prefix=${INSTALL}"

  run_logged 01_python setup_python
  run_logged 01b_eigen setup_eigen
  run_logged 02_uuid build_uuid
  run_logged 03_boost build_boost
  run_logged 04_tinyxml2 build_tinyxml2
  run_logged 05_console_bridge build_console_bridge
  run_logged 06_poco build_poco
  run_logged 07_fetch_ros fetch_ros
  run_logged 08_catkin build_ros
  run_logged 09_tree write_final_tree

  [[ -f "${INSTALL}/lib/libnodeletlib.so" ]] || fail "缺少 libnodeletlib.so"
  [[ -f "${INSTALL}/lib/libroscpp.so" ]] || fail "缺少 libroscpp.so"
  [[ -x "${INSTALL}/bin/rosmaster" ]] || fail "缺少 bin/rosmaster"

  log "==== BUILD SUCCESS ===="
  log "Install tree: ${INSTALL}"
  log "下一步: $0 package   （应用层: app/*/make.sh）"
}

# -----------------------------------------------------------------------------
# package：抽出可拷走的 ROS 运行目录 custom_mini_runtime/
# -----------------------------------------------------------------------------
# 内容：
#   bin/   rosmaster（shebang 改成 /usr/bin/env python3）
#   lib/   按关键 ROS 库 ldd 扫到的非系统库
#   python/  rosmaster 需要的模块 + python_compat
#   软链接  README.md / run.sh / env.sh → doc/custom_mini/
cmd_package() {
  local nodeletlib="${INSTALL}/lib/libnodeletlib.so"
  local roscpplib="${INSTALL}/lib/libroscpp.so"
  local classloader="${INSTALL}/lib/libclass_loader.so"
  [[ -f "${nodeletlib}" ]] || fail "缺少 ${nodeletlib}，请先: $0 build"
  [[ -f "${roscpplib}" ]] || fail "缺少 ${roscpplib}，请先: $0 build"
  [[ -x "${INSTALL}/bin/rosmaster" ]] || fail "缺少 ${INSTALL}/bin/rosmaster，请先: $0 build"
  [[ -d "${DOC}" ]] || fail "缺少 ${DOC}"

  echo "==== package → ${RUNTIME} ===="
  rm -rf "${RUNTIME}"
  mkdir -p "${RUNTIME}/bin" "${RUNTIME}/lib" "${RUNTIME}/python"

  local lib
  while read -r lib; do
    copy_lib "${lib}" "${RUNTIME}/lib"
  done < <(
    {
      collect_nonsystem_libs "${nodeletlib}"
      collect_nonsystem_libs "${roscpplib}"
      collect_nonsystem_libs "${classloader}"
      printf '%s\n' "${nodeletlib}" "${roscpplib}" "${classloader}"
    } | sort -u
  )

  sed '1s|^#!.*|#!/usr/bin/env python3|' "${INSTALL}/bin/rosmaster" > "${RUNTIME}/bin/rosmaster"
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
  link_doc env.sh
  chmod +x "${DOC}/run.sh"

  echo "==== PACKAGE SUCCESS ===="
  echo "runtime: ${RUNTIME}"
  echo "下一步: $0 run   （应用层: app/*/make.sh x86 && install，再 app_runtime/run.sh）"
}

# exec：用 run.sh 替换当前进程（进入交互式 ROS 环境 shell）。
cmd_run() {
  [[ -e "${RUNTIME}/run.sh" ]] || fail "未找到 ${RUNTIME}/run.sh，请先: $0 package"
  exec "${RUNTIME}/run.sh"
}

# 只删编译中间文件。install / runtime / logs 都留着，避免误清可运行产物。
cmd_clean() {
  echo "删除 ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
  echo "clean 完成（保留 ${INSTALL} 与 ${RUNTIME}）"
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

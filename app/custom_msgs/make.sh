#!/usr/bin/env bash
# =============================================================================
# app/custom_msgs/make.sh
# -----------------------------------------------------------------------------
# 用 gencpp 从 .msg 生成 C++ 头文件（不依赖 catkin 包构建）。
#
#   ./make.sh           编译 msgs → build/include/custom_msgs/
#   ./make.sh install   安装头文件 → ../common_include/custom_msgs/
#   ./make.sh clean     删除 build/
#   ./make.sh help
# =============================================================================
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${PKG}/../.." && pwd)"
ROS_INSTALL="${ROS_INSTALL:-${ROOT}/custom_mini_install}"
BUILD="${PKG}/build"
GEN_OUT="${BUILD}/include/custom_msgs"
COMMON_INCLUDE="${PKG}/../common_include/custom_msgs"
MSG_DIR="${PKG}/msg"
PKG_NAME="custom_msgs"
GEN_CPP="${ROS_INSTALL}/lib/gencpp/gen_cpp.py"
TEMPLATE_DIR="${ROS_INSTALL}/share/gencpp"
PY_SITE="${ROS_INSTALL}/lib/python3.12/site-packages"
PY_DIST="${ROS_INSTALL}/lib/python3/dist-packages"
PY_COMPAT="${ROOT}/python_compat"

fail() { echo "ERROR: $*" >&2; exit 1; }

cmd_help() {
  cat <<EOF
用法: $0 [build|install|clean|help]

  (默认)/build  用 gencpp 生成头文件 → ${GEN_OUT}/
  install       将头文件安装到 ${COMMON_INCLUDE}/
  clean         删除 ${BUILD}
  help          显示本说明
EOF
}

prepare_env() {
  [[ -f "${GEN_CPP}" ]] || fail "缺少 gencpp: ${GEN_CPP}（请先 ./custom_mini.sh build）"
  [[ -d "${TEMPLATE_DIR}" ]] || fail "缺少 gencpp 模板: ${TEMPLATE_DIR}"
  export PYTHONPATH="${PY_COMPAT}:${PY_SITE}:${PY_DIST}${PYTHONPATH:+:${PYTHONPATH}}"
  export PYTHONNOUSERSITE=1
}

cmd_build() {
  prepare_env
  echo "==== custom_msgs build ===="
  mkdir -p "${GEN_OUT}"
  local msg
  local count=0
  for msg in "${MSG_DIR}"/*.msg; do
    [[ -f "${msg}" ]] || continue
    echo "Generating: ${PKG_NAME}/$(basename "${msg}")"
    python3 "${GEN_CPP}" "${msg}" \
      -p "${PKG_NAME}" \
      -o "${GEN_OUT}" \
      -e "${TEMPLATE_DIR}" \
      -I "${PKG_NAME}:${MSG_DIR}"
    count=$((count + 1))
  done
  [[ "${count}" -gt 0 ]] || fail "未找到 ${MSG_DIR}/*.msg"
  echo "DONE: ${GEN_OUT}/ ($(ls -1 "${GEN_OUT}"/*.h 2>/dev/null | wc -l) headers)"
  echo "下一步: $0 install"
}

cmd_install() {
  [[ -d "${GEN_OUT}" ]] || fail "缺少 ${GEN_OUT}，请先: $0"
  local hdr
  local n=0
  mkdir -p "${COMMON_INCLUDE}"
  for hdr in "${GEN_OUT}"/*.h; do
    [[ -f "${hdr}" ]] || continue
    cp -a "${hdr}" "${COMMON_INCLUDE}/"
    echo "INSTALLED: ${COMMON_INCLUDE}/$(basename "${hdr}")"
    n=$((n + 1))
  done
  [[ "${n}" -gt 0 ]] || fail "未找到可安装的头文件"
}

cmd_clean() {
  echo "删除 ${BUILD}"
  rm -rf "${BUILD}"
}

case "${1:-build}" in
  build) cmd_build ;;
  install) cmd_install ;;
  clean) cmd_clean ;;
  help|-h|--help) cmd_help ;;
  *) echo "未知命令: $1" >&2; cmd_help; exit 1 ;;
esac

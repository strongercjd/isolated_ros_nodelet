#!/usr/bin/env bash
# =============================================================================
# custom_mini_runtime/env.sh
# -----------------------------------------------------------------------------
# 把本 runtime 的 bin/lib/python 注入当前 shell。由 run.sh source，也可手动:
#   source custom_mini_runtime/env.sh
# =============================================================================

# 被 source 时 BASH_SOURCE 指向本文件；经软链接时用 pwd -L 留在 runtime。
_CUSTOM_MINI_ENV_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -L)"

export LD_LIBRARY_PATH="${_CUSTOM_MINI_ENV_HERE}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${_CUSTOM_MINI_ENV_HERE}/python${PYTHONPATH:+:${PYTHONPATH}}"
export PATH="${_CUSTOM_MINI_ENV_HERE}/bin:${PATH}"
export PKG_CONFIG_PATH="${_CUSTOM_MINI_ENV_HERE}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
export CMAKE_PREFIX_PATH="${_CUSTOM_MINI_ENV_HERE}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_DISTRO="${ROS_DISTRO:-noetic}"

export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1

export ROS_HOME="${_CUSTOM_MINI_ENV_HERE}/.ros"
mkdir -p "${ROS_HOME}/log"

unset _CUSTOM_MINI_ENV_HERE

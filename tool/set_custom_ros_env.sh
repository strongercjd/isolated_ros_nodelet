#!/bin/bash
# 检测当前脚本是否被 source 执行
# 方法：比较 ${BASH_SOURCE[0]} 和 $0
# 如果相同，说明是直接执行（./set_custom_ros_env.sh 或 bash set_custom_ros_env.sh）
# 如果不同，说明是被 source 调用

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "错误：此脚本必须使用 source 执行，而不能直接运行。"
    echo "正确用法：source set_custom_ros_env.sh  或  . set_custom_ros_env.sh"
    exit 1   # 直接执行时退出子 shell（不会影响当前终端）
fi

# ---------- 以下是环境变量设置（仅在 source 时执行） ----------
export LD_LIBRARY_PATH=$PWD/custom_mini_install/lib
export PYTHONPATH=$PWD/python_compat:$PWD/custom_mini_install/lib/python3.12/site-packages:$PWD/custom_mini_install/lib/python3/dist-packages
export PATH=$PWD/custom_mini_install/bin:$PATH

echo "环境变量已成功设置。"
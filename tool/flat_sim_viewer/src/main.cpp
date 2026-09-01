// flat_sim_viewer —— SLAM 建图查看工具入口（必有 UI，无 headless 形态）
#include <QApplication>

#include <cstdio>
#include <cstdlib>
#include <string>

#include "app/ViewerWindow.h"

int main(int argc, char** argv) {
  std::string bagPath;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      std::puts(
          "flat_sim_viewer —— 订阅 /slam2d/* 实时查看 SLAM 建图（Qt6），或回放本地 bag\n"
          "\n"
          "用法: flat_sim_viewer [-h] [--bag <file.bag>]\n"
          "\n"
          "  无参数启动实时模式；无 rosmaster 时以未连接状态启动并在状态栏提示重试\n"
          "  --bag  启动后直接进入回放模式加载该日志（map_log_nodelet 录制）\n"
          "  按键: v 恢复视图跟随  |  空格 播放/暂停  |  ←/→ 单步帧  |  ESC / q 退出\n"
          "  鼠标: 滚轮缩放（锚定光标）| 左键拖拽平移（脱离跟随）\n"
          "  日常请通过 tool/flat_sim_viewer/run_viewer.sh 启动");
      return 0;
    }
    if (a == "--bag") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "--bag 需要一个文件参数\n");
        return 1;
      }
      bagPath = argv[++i];
      continue;
    }
    std::fprintf(stderr, "未知参数: %s（支持 -h / --help / --bag <file>）\n", a.c_str());
    return 1;
  }

  if (!qEnvironmentVariableIsSet("DISPLAY") &&
      !qEnvironmentVariableIsSet("WAYLAND_DISPLAY")) {
    std::fprintf(stderr,
                 "[flat_sim_viewer] 无显示器（DISPLAY 未设置）。"
                 "查看器必须带 UI 运行。\n");
    return 1;
  }

  QApplication app(argc, argv);
  flat_sim_viewer::ViewerWindow win(nullptr, bagPath);
  win.show();
  return app.exec();
}

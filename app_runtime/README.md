# app_runtime — 应用层

```bash
./custom_mini.sh build && ./custom_mini.sh package

(cd app/talker_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/listener_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/custom_ros_nodelet && ./make.sh x86 && ./make.sh install)

# 终端 1
./custom_mini.sh run

# 终端 2
./app_runtime/run.sh
```

| 路径 | 作用 |
|------|------|
| `bin/custom_ros_nodelet_manager` | 读 `plugins.json`，加载插件 |
| `lib/libtalker_nodelet.so` | 发送端（`app/talker_nodelet`） |
| `lib/liblistener_nodelet.so` | 接收端（`app/listener_nodelet`） |
| `plugins.json` | 插件列表 |
| `run.sh` | 检测 ROS Master 后启动 manager |

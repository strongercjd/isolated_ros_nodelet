// flat_sim_teleop —— 键盘遥控 flat_sim 机器人（发布 /cmd_vel）
#include "TeleopConfig.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace {

enum class Key { None, Up, Down, Left, Right, Quit };

struct TermiosGuard {
  termios saved{};
  bool active = false;

  ~TermiosGuard() { restore(); }

  bool enableRaw() {
    if (!isatty(STDIN_FILENO)) return false;
    if (tcgetattr(STDIN_FILENO, &saved) != 0) return false;
    termios raw = saved;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
    active = true;
    return true;
  }

  void restore() {
    if (active) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved);
      active = false;
    }
  }
};

Key readKey() {
  unsigned char c = 0;
  const ssize_t n = read(STDIN_FILENO, &c, 1);
  if (n <= 0) return Key::None;

  if (c == 3) return Key::Quit;  // Ctrl+C（raw 模式下需自行处理）
  if (c == 'q' || c == 'Q') return Key::Quit;

  if (c != 0x1b) return Key::None;

  unsigned char seq[2] = {0, 0};
  if (read(STDIN_FILENO, &seq[0], 1) <= 0) return Key::None;
  if (seq[0] != '[') return Key::None;
  if (read(STDIN_FILENO, &seq[1], 1) <= 0) return Key::None;

  switch (seq[1]) {
    case 'A':
      return Key::Up;
    case 'B':
      return Key::Down;
    case 'C':
      return Key::Right;
    case 'D':
      return Key::Left;
    default:
      return Key::None;
  }
}

void applyKey(Key key, geometry_msgs::Twist& twist) {
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  switch (key) {
    case Key::Up:
      twist.linear.x = FLAT_SIM_TELEOP_LINEAR_SPEED;
      break;
    case Key::Down:
      twist.linear.x = -FLAT_SIM_TELEOP_LINEAR_SPEED;
      break;
    case Key::Left:
      twist.angular.z = FLAT_SIM_TELEOP_ANGULAR_SPEED;
      break;
    case Key::Right:
      twist.angular.z = -FLAT_SIM_TELEOP_ANGULAR_SPEED;
      break;
    default:
      break;
  }
}

void printHelp(const char* robot) {
  std::printf(
      "\nflat_sim 键盘遥控（机器人：%s）\n"
      "  ↑  前进 %.2f m/s\n"
      "  ↓  后退 %.2f m/s\n"
      "  ←  逆时针 %.2f rad/s\n"
      "  →  顺时针 %.2f rad/s\n"
      "  q / Ctrl+C  退出并发送零速\n\n",
      robot, FLAT_SIM_TELEOP_LINEAR_SPEED, FLAT_SIM_TELEOP_LINEAR_SPEED,
      FLAT_SIM_TELEOP_ANGULAR_SPEED, FLAT_SIM_TELEOP_ANGULAR_SPEED);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "flat_sim_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string robot = "mycar";
  pnh.param("robot", robot, robot);

  const std::string topic = "/" + robot + "/cmd_vel";
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic, 1);

  if (!isatty(STDIN_FILENO)) {
    std::fprintf(stderr, "错误：需要在终端中运行（stdin 必须是 TTY）。\n");
    return 1;
  }

  TermiosGuard term;
  if (!term.enableRaw()) {
    std::fprintf(stderr, "错误：无法设置终端 raw 模式。\n");
    return 1;
  }

  printHelp(robot.c_str());

  ros::Rate rate(20.0);
  Key active = Key::None;
  geometry_msgs::Twist cmd;

  while (ros::ok()) {
    const Key key = readKey();
    if (key == Key::Quit) break;
    // 有键按下（含终端 repeat）则运动；本周期无输入则停（松开即零速）
    active = (key != Key::None && key != Key::Quit) ? key : Key::None;

    applyKey(active, cmd);
    pub.publish(cmd);

    ros::spinOnce();
    rate.sleep();
  }

  cmd = geometry_msgs::Twist();
  pub.publish(cmd);
  return 0;
}

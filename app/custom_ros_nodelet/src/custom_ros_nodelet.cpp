/**
 * 用 JSON 描述要加载的 Nodelet，经 class_loader 按 .so 路径直接实例化。
 * 不走 pluginlib / rospack / package.xml / nodelet_plugins.xml。
 *
 * 用法: custom_ros_nodelet_manager <plugins.json>
 */

#include "custom_ros_nodelet.h"

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace
{

std::string dirnameOf(const std::string& path)
{
  const std::string::size_type pos = path.find_last_of('/');
  if (pos == std::string::npos)
  {
    return ".";
  }
  if (pos == 0)
  {
    return "/";
  }
  return path.substr(0, pos);
}

bool isAbsolute(const std::string& path)
{
  return !path.empty() && path[0] == '/';
}

std::string joinPath(const std::string& a, const std::string& b)
{
  if (b.empty())
  {
    return a;
  }
  if (isAbsolute(b) || a.empty() || a == ".")
  {
    return b;
  }
  if (a.back() == '/')
  {
    return a + b;
  }
  return a + "/" + b;
}

// 日志配置（plugins.json 顶层 "log" 字段）
struct LogSpec
{
  bool enabled = true;    // false 时保持打印到终端
  std::string dir = "log";                    // 相对 plugins.json 所在目录
  std::string file = "custom_ros_nodelet.log";
  std::string format = "[${severity}] [${time:%m-%d %H:%M:%S}]: ${message}";
  bool color = false;     // 文件日志默认关 ANSI 颜色
};

// 递归创建目录（mkdir -p）
bool makeDirs(const std::string& path)
{
  if (path.empty() || path == "." || path == "/")
  {
    return true;
  }
  std::string acc;
  std::string::size_type start = 0;
  if (path[0] == '/')
  {
    acc = "/";
    start = 1;
  }
  while (start < path.size())
  {
    const std::string::size_type slash = path.find('/', start);
    const std::string seg = (slash == std::string::npos) ? path.substr(start) : path.substr(start, slash - start);
    if (!seg.empty())
    {
      acc = acc.empty() ? seg : (acc.back() == '/' ? acc + seg : acc + "/" + seg);
      if (mkdir(acc.c_str(), 0755) != 0 && errno != EEXIST)
      {
        return false;
      }
    }
    if (slash == std::string::npos)
    {
      break;
    }
    start = slash + 1;
  }
  return true;
}

// 日志配置的生效分两代进程完成：
//   第一代：setenv（ROSCONSOLE_FORMAT=${time:FORMAT} 月日时分秒格式 / NO_COLOR 关色码）
//           后 execv 自我重启。必须 re-exec 是因为 rosconsole 的 initialize() 在库的
//   静态初始化阶段（main 之前）就可能已被触发并定格格式/颜色，进程内补设环境变量无效；
//   重启后新进程从静态初始化起即读到新环境。
//   第二代（哨兵变量已设）：dup2 stdout/stderr → 日志文件，manager 与全部动态库的
//   输出（rosconsole 的 fprintf、printf、std::cout）都写入文件。
void applyLogSpec(const LogSpec& spec, const std::string& json_dir, char** argv)
{
  if (!spec.enabled)
  {
    return;
  }

  if (getenv("CUSTOM_ROS_NODELET_LOG_ENV") == NULL)
  {
    // 第一代进程：注入环境后重启自身（execv 成功不返回）
    setenv("CUSTOM_ROS_NODELET_LOG_ENV", "1", 1);
    setenv("ROSCONSOLE_FORMAT", spec.format.c_str(), 1);
    if (spec.color)
    {
      unsetenv("NO_COLOR");
    }
    else
    {
      setenv("NO_COLOR", "1", 1);
    }
    fprintf(stderr, "apply ROSCONSOLE_FORMAT, re-exec %s ...\n", argv[0]);
    fflush(stderr);
    if (execv("/proc/self/exe", argv) == -1)
    {
      fprintf(stderr, "re-exec failed (%s), continue with default log format\n",
              strerror(errno));
      // 继续做 dup2：至少日志仍写入文件
    }
    else
    {
      return; // 不可达，保险
    }
  }

  std::string dir = isAbsolute(spec.dir) ? spec.dir : joinPath(json_dir, spec.dir);
  if (!makeDirs(dir))
  {
    fprintf(stderr, "cannot create log dir %s: %s (logs stay on terminal)\n",
            dir.c_str(), strerror(errno));
    return;
  }
  const std::string path = joinPath(dir, spec.file);
  const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0)
  {
    fprintf(stderr, "cannot open log file %s: %s (logs stay on terminal)\n",
            path.c_str(), strerror(errno));
    return;
  }

  fprintf(stderr, "==== logs redirect to %s ====\n", path.c_str());
  fflush(stderr);
  if (dup2(fd, STDOUT_FILENO) < 0 || dup2(fd, STDERR_FILENO) < 0)
  {
    fprintf(stderr, "dup2 failed: %s (logs stay on terminal)\n", strerror(errno));
    close(fd);
    return;
  }
  close(fd);
}

struct PluginSpec
{
  std::string name;
  std::string class_id;
  std::string library;
  nodelet::M_string remap;
  nodelet::V_string args;
  bool enabled;
};

class JsonPluginFactory
{
public:
  explicit JsonPluginFactory(const std::string& json_path)
  {
    std::ifstream in(json_path.c_str());
    if (!in)
    {
      throw std::runtime_error("cannot open JSON: " + json_path);
    }
    nlohmann::json root;
    in >> root;
    if (!root.is_object())
    {
      throw std::runtime_error("JSON root must be an object");
    }

    const int version = root.value("version", 1);
    if (version < 1)
    {
      throw std::runtime_error("unsupported plugins.json version");
    }

    node_name_ = root.value("node", std::string("custom_ros_nodelet_manager"));
    const std::string json_dir = dirnameOf(json_path);
    json_dir_ = json_dir;
    if (root.contains("log") && root["log"].is_object())
    {
      const nlohmann::json& j = root["log"];
      log_spec_.enabled = j.value("enabled", log_spec_.enabled);
      log_spec_.dir = j.value("dir", log_spec_.dir);
      log_spec_.file = j.value("file", log_spec_.file);
      log_spec_.format = j.value("format", log_spec_.format);
      log_spec_.color = j.value("color", log_spec_.color);
    }
    std::string library_dir = "lib";
    if (root.contains("defaults") && root["defaults"].is_object())
    {
      library_dir = root["defaults"].value("library_dir", library_dir);
    }
    const std::string library_base = joinPath(json_dir, library_dir);

    if (!root.contains("plugins") || !root["plugins"].is_array())
    {
      throw std::runtime_error("JSON must contain a \"plugins\" array");
    }

    for (const auto& item : root["plugins"])
    {
      if (!item.is_object())
      {
        throw std::runtime_error("each plugin must be an object");
      }
      PluginSpec spec;
      spec.name = item.value("name", std::string());
      spec.class_id = item.value("class", std::string());
      spec.library = item.value("library", std::string());
      spec.enabled = item.value("enabled", true);
      if (spec.name.empty() || spec.class_id.empty() || spec.library.empty())
      {
        throw std::runtime_error("plugin requires name, class, library");
      }
      if (item.contains("remap") && item["remap"].is_object())
      {
        for (auto it = item["remap"].begin(); it != item["remap"].end(); ++it)
        {
          spec.remap[it.key()] = it.value().get<std::string>();
        }
      }
      if (item.contains("args") && item["args"].is_array())
      {
        for (const auto& a : item["args"])
        {
          spec.args.push_back(a.get<std::string>());
        }
      }
      if (!isAbsolute(spec.library))
      {
        spec.library = joinPath(library_base, spec.library);
      }
      specs_.push_back(spec);
    }
  }

  const std::string& nodeName() const
  {
    return node_name_;
  }

  const LogSpec& logSpec() const
  {
    return log_spec_;
  }

  const std::string& jsonDir() const
  {
    return json_dir_;
  }

  const std::vector<PluginSpec>& specs() const
  {
    return specs_;
  }

  boost::shared_ptr<nodelet::Nodelet> create(const std::string& class_id)
  {
    const PluginSpec* spec = 0;
    for (size_t i = 0; i < specs_.size(); ++i)
    {
      if (specs_[i].class_id == class_id)
      {
        spec = &specs_[i];
        break;
      }
    }
    if (!spec)
    {
      throw std::runtime_error("unknown plugin class in JSON: " + class_id);
    }

    boost::shared_ptr<class_loader::ClassLoader>& loader = loaders_[spec->library];
    if (!loader)
    {
      loader.reset(new class_loader::ClassLoader(spec->library, false));
    }
    return loader->createInstance<nodelet::Nodelet>(spec->class_id);
  }

private:
  std::string node_name_;
  std::string json_dir_;
  LogSpec log_spec_;
  std::vector<PluginSpec> specs_;
  std::map<std::string, boost::shared_ptr<class_loader::ClassLoader> > loaders_;
};

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: " << argv[0] << " <plugins.json>\n";
    return 1;
  }

  JsonPluginFactory factory(argv[1]);

  // 日志重定向与格式必须在 ros::init（首次 rosconsole 初始化）之前生效
  applyLogSpec(factory.logSpec(), factory.jsonDir(), argv);

  ros::init(argc, argv, factory.nodeName());

  nodelet::Loader loader(boost::bind(&JsonPluginFactory::create, &factory, boost::placeholders::_1));

  for (size_t i = 0; i < factory.specs().size(); ++i)
  {
    const PluginSpec& spec = factory.specs()[i];
    if (!spec.enabled)
    {
      ROS_INFO("skip disabled plugin %s", spec.name.c_str());
      continue;
    }
    if (!loader.load(spec.name, spec.class_id, spec.remap, spec.args))
    {
      ROS_FATAL("Failed to load %s (%s) from %s",
                spec.name.c_str(), spec.class_id.c_str(), spec.library.c_str());
      return 1;
    }
    ROS_INFO("loaded %s class=%s library=%s",
             spec.name.c_str(), spec.class_id.c_str(), spec.library.c_str());
  }

  ros::spin();
  return 0;
}

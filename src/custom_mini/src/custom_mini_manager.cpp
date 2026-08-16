/**
 * 用 JSON 描述要加载的 Nodelet，经 class_loader 按 .so 路径直接实例化。
 * 不走 pluginlib / rospack / package.xml / nodelet_plugins.xml。
 *
 * 用法: custom_mini_manager <plugins.json>
 */
#include <class_loader/class_loader.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <nlohmann/json.hpp>

#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <vector>

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

    node_name_ = root.value("node", std::string("custom_mini_manager"));
    const std::string json_dir = dirnameOf(json_path);
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

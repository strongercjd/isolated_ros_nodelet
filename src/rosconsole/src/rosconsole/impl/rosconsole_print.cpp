/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/console.h"
#define ROSCONSOLE_CONSOLE_IMPL_EXPORTS
#include "ros/console_impl.h"

#include <map>
#include <string>
#include <boost/thread/mutex.hpp>

namespace ros
{
namespace console
{
namespace impl
{

LogAppender* rosconsole_print_appender = 0;

// logger 名驻留表：handle = 指向驻留字符串的指针（进程生命周期常驻，
// 每个名字仅一次分配）。使 ${logger} token 能区分同进程内各 nodelet 的日志来源
namespace
{
boost::mutex g_handle_mutex;
std::map<std::string, const std::string*> g_handles;
}

void initialize()
{}

void print(void* handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  ::ros::console::backend::print(handle, level, str, file, function, line);
  if(rosconsole_print_appender)
  {
    rosconsole_print_appender->log(level, str, file, function, line);
  }
}

bool isEnabledFor(void* handle, ::ros::console::Level level)
{
  return level != ::ros::console::levels::Debug;
}

void* getHandle(const std::string& name)
{
  boost::mutex::scoped_lock lock(g_handle_mutex);
  std::map<std::string, const std::string*>::iterator it = g_handles.find(name);
  if (it != g_handles.end())
  {
    return const_cast<void*>(static_cast<const void*>(it->second));
  }
  const std::string* held = new std::string(name);
  g_handles[name] = held;
  return const_cast<void*>(static_cast<const void*>(held));
}

std::string getName(void* handle)
{
  if (!handle)
  {
    return "";
  }
  std::string name = *static_cast<const std::string*>(handle);
  // 去掉 rosconsole 的 logger 前缀链（"ros.xxx_nodelet./node" → "/node"），
  // 使 ${logger} 标签只显示 nodelet 名；无前缀的名字（如 "ros"）原样返回
  size_t pos = name.rfind('.');
  return pos == std::string::npos ? name : name.substr(pos + 1);
}

void register_appender(LogAppender* appender)
{
  rosconsole_print_appender = appender;
}

void deregister_appender(LogAppender* appender){
  if(rosconsole_print_appender == appender)
  {
    rosconsole_print_appender = 0;
  }
}

void shutdown()
{}

bool get_loggers(std::map<std::string, levels::Level>& loggers)
{
  return true;
}

bool set_logger_level(const std::string& name, levels::Level level)
{
  return false;
}

} // namespace impl
} // namespace console
} // namespace ros

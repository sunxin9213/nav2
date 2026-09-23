// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_util/node_utils.hpp"
// 本文件实现节点相关的实用工具函数：
// - sanitize_node_name: 清洗节点名中的非法字符
// - add_namespaces: 拼接两级命名空间得到绝对命名空间
// - time_to_string: 用系统时间生成伪随机数字串
// - generate_internal_node_name / generate_internal_node: 生成内部辅助节点
#include <chrono>
#include <string>
#include <algorithm>
#include <cctype>

using std::chrono::high_resolution_clock;
using std::to_string;
using std::string;
using std::replace_if;
using std::isalnum;

namespace nav2_util
{

string sanitize_node_name(const string & potential_node_name)
{
  // ROS2 节点名只允许字母、数字和下划线，这里先把输入复制一份再原地清洗
  string node_name(potential_node_name);
  // read this as `replace` characters in `node_name` `if` not alphanumeric.
  // replace with '_'
  // 语义：遍历整个字符串，凡是"不是字母数字"的字符都替换为 '_'，
  // isalnum 返回 0 表示非法字符（注意空格、'-'、'.' 等都会被替换）
  replace_if(
    begin(node_name), end(node_name),
    [](auto c) {return !isalnum(c);},
    '_');
  return node_name;
}

string add_namespaces(const string & top_ns, const string & sub_ns)
{
  // 情况一：顶层命名空间非空且以 '/' 结尾，说明分隔符已经存在
  if (!top_ns.empty() && top_ns.back() == '/') {
    if (top_ns.front() == '/') {
      // 已经是绝对命名空间（以 '/' 开头），直接拼接子命名空间即可
      return top_ns + sub_ns;
    } else {
      // 是相对命名空间，补一个前导 '/' 变成绝对命名空间
      return "/" + top_ns + sub_ns;
    }
  }

  // 情况二：顶层命名空间为空或没有以 '/' 结尾，需要手动补上分隔符 '/'
  return top_ns + "/" + sub_ns;
}

std::string time_to_string(size_t len)
{
  string output(len, '0');  // prefill the string with zeros
  // 取高精度时钟自 epoch 以来的计数值并转为字符串，作为"伪随机"数字来源
  auto timepoint = high_resolution_clock::now();
  auto timecount = timepoint.time_since_epoch().count();
  auto timestring = to_string(timecount);
  if (timestring.length() >= len) {
    // if `timestring` is shorter, put it at the end of `output`
    // 时间串比目标长：截取时间串的"末尾" len 位填入（低位数字变化最快，随机性最好）
    output.replace(
      0, len,
      timestring,
      timestring.length() - len, len);
  } else {
    // if `output` is shorter, just copy in the end of `timestring`
    // 时间串比目标短：右对齐放入，前面保留预填充的 '0'
    output.replace(
      len - timestring.length(), timestring.length(),
      timestring,
      0, timestring.length());
  }
  return output;
}

std::string generate_internal_node_name(const std::string & prefix)
{
  // 内部节点名 = 清洗过的前缀 + '_' + 8 位时间数字串，保证系统内名称唯一
  return sanitize_node_name(prefix) + "_" + time_to_string(8);
}

rclcpp::Node::SharedPtr generate_internal_node(const std::string & prefix)
{
  // 内部辅助节点不需要对外提供参数服务，也不发布参数事件，可节省资源
  auto options =
    rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    // 通过命令行重映射参数把节点名改为生成的唯一名字（"__node:=xxx"），
    // 末尾的 "--" 用于终结 ROS 参数解析，防止后续参数被误解析
    .arguments({"--ros-args", "-r", "__node:=" + generate_internal_node_name(prefix), "--"});
  // 构造时传入的 "_" 只是占位符，真实节点名已被上面的重映射覆盖
  return rclcpp::Node::make_shared("_", options);
}

}  // namespace nav2_util

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

#include "nav2_util/string_utils.hpp"
#include <string>

using std::string;

namespace nav2_util
{
// 本文件实现字符串工具：去除前导斜杠与按分隔符切分字符串

std::string strip_leading_slash(const string & in)
{
  string out = in;

  // ROS 话题名常以 '/' 开头（绝对名称），某些 API 需要相对名称时去掉首个 '/'
  if ((!in.empty()) && (in[0] == '/')) {
    out.erase(0, 1);
  }

  return out;
}

Tokens split(const string & tokenstring, char delimiter)
{
  Tokens tokens;

  // 经典的手写切分循环：
  // 不断从 current_pos 起查找下一个分隔符，截取两分隔符之间的子串
  size_t current_pos = 0;
  size_t pos = 0;
  while ((pos = tokenstring.find(delimiter, current_pos)) != string::npos) {
    tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
    current_pos = pos + 1;  // 跳过分隔符本身
  }
  // 收尾：追加最后一段（其后不再有分隔符）
  tokens.push_back(tokenstring.substr(current_pos));
  return tokens;
}

}  // namespace nav2_util

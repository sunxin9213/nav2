/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * author: Dave Hershberger
 */
// ============================================================================
// 中文注释：数组字符串解析器（array_parser）实现文件
//
// 【文件功能】
// 本文件实现 parseVVF()——把形如 "[[1.0, 2.0], [3.3, 4.4, 5.5]]" 的
// 二维浮点数组字符串解析为 std::vector<std::vector<float>>。
// 在 Nav2 中主要用于解析 YAML 中的 footprint（足迹多边形）参数：
//   footprint: "[[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]"
// 解析结果由 makeFootprintFromString() 转为足迹顶点。
//
// 【实现原理】
// 基于字符流（stringstream）逐字符状态机：
//   - '[' ：深度 +1，超过 2 层报错；开始新的一维数组；
//   - ']' ：深度 -1，配对多于 '[' 报错；深度回到 1 时收尾一维数组；
//   - ','/空格/tab：分隔符，跳过；
//   - 其他：必须是数字，且只允许出现在深度 2（最内层）。
// ============================================================================

#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>

namespace nav2_costmap_2d
{

/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
// 中文注释：解析二维 float 数组字符串。
// input：待解析字符串；error_return：出错时返回错误描述（空串表示成功）
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)
{
  std::vector<std::vector<float>> result;

  std::stringstream input_ss(input);   // 字符流（支持 >> 提取数字）
  int depth = 0;                       // 当前括号嵌套深度（合法范围 0~2）
  std::vector<float> current_vector;   // 正在解析的一维数组
  while (!!input_ss && !input_ss.eof()) {
    // peek() 只看不取：根据下一字符分派处理
    switch (input_ss.peek()) {
      case EOF:
        break;                         // 流结束
      case '[':
        // 开括号：深度+1，开始新的一维数组
        depth++;
        if (depth > 2) {
          // 只支持两层嵌套（二维数组）
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();                // 消费该字符
        current_vector.clear();        // 准备接收新的数字序列
        break;
      case ']':
        // 闭括号：深度-1
        depth--;
        if (depth < 0) {
          // ']' 比 '[' 多：语法错误
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          // 从最内层退回到第一层：一维数组完成，收入结果
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        // 分隔符与空白：直接跳过
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        // 其他字符：应当是数字的一部分
        if (depth != 2) {
          // 数字只允许出现在最内层（深度 2）
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        float value;
        input_ss >> value;             // 用流提取数字
        if (!!input_ss) {
          current_vector.push_back(value);   // 提取成功则加入当前数组
        }
        break;
    }
  }

  // 括号不配对（有 '[' 未闭合）→ 报错；否则成功（错误串为空）
  if (depth != 0) {
    error_return = "Unterminated vector string.";
  } else {
    error_return = "";
  }

  return result;
}

}  // end namespace nav2_costmap_2d

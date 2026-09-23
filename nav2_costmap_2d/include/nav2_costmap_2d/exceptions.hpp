/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef NAV2_COSTMAP_2D__EXCEPTIONS_HPP_
#define NAV2_COSTMAP_2D__EXCEPTIONS_HPP_
// ============================================================================
// 中文注释：代价地图异常体系头文件
//
// 【文件功能】
// 定义 nav2_costmap_2d 的自定义异常（均继承 std::runtime_error）：
//   - Costmap2DCLIException：非法命令行/服务参数；
//   - IllegalPoseException：给定位姿落在代价地图范围之外
//     （碰撞检测时抛出，见 costmap_topic_collision_checker）；
//   - CollisionCheckerException：碰撞检测运行时错误
//     （如地图/足迹尚不可用）。
// Nav2 惯例：用继承自 runtime_error 的具名异常类区分错误类别，
// 便于调用方按类型捕获并做不同处理（安全优先时统一视为碰撞）。
// ============================================================================

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_costmap_2d
{
/**
 * @class CollisionCheckerException
 * @brief Exceptions thrown if collision checker determines a pose is in
 * collision with the environment costmap
 */
class CollisionCheckerException : public std::runtime_error
{
public:
  explicit CollisionCheckerException(const std::string description)
  : std::runtime_error(description) {}
};

/**
 * @class IllegalPoseException
 * @brief Thrown when CollisionChecker encounters a fatal error
 */
class IllegalPoseException : public CollisionCheckerException
{
public:
  IllegalPoseException(const std::string name, const std::string description)
  : CollisionCheckerException(description), name_(name) {}
  std::string getCriticName() const {return name_;}

protected:
  std::string name_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__EXCEPTIONS_HPP_

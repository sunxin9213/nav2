// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>
#include <string>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 * （运动模型抽象基类：MPPI 在轨迹推演（rollout）时用运动学模型
 *  把控制序列映射为整车速度序列。predict() 对所有 batch×time_steps
 *  的采样做向量化赋值，是优化循环中的热路径。）
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  // 基类默认实现：直接把控制序列当作整车速度（一阶直通模型）。
  // 利用 xtensor 视图做批量对齐：state.vx 的第 t 步取 cvx 的第 t-1 步，
  // 即"控制量在下一步生效"，避免原地赋值时数据被覆盖（noalias 关闭别名检查提速）
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT
    // 水平速度：vx[:, 1:] = cvx[:, :-1]，首列保留当前实际速度
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    // 角速度同理：wz[:, 1:] = cwz[:, :-1]
    xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
      xt::view(state.cwz, xt::all(), xt::range(0, -1));

    // 全向模型额外处理侧向速度 vy
    if (isHolonomic()) {
      xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
        xt::view(state.cvy, xt::all(), xt::range(0, -1));
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  // 硬约束钩子：基类默认不做任何约束（差速/全向无需额外处理）
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 * （阿克曼运动模型：车不侧滑，转弯半径存在机械下限。
 *  applyConstraints 在每步控制上强制满足 r = |vx|/|wz| >= min_turning_r_，
 *  即 |wz| <= |vx| / min_turning_r_，超限的角速度被削顶。）
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  // 读取最小转弯半径参数（<name>.AckermannConstraints.min_turning_r）
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;  // 阿克曼结构无法侧向平移
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & wz = control_sequence.wz;
    auto abs_vx = xt::fabs(control_sequence.vx);  // |vx|：线速度绝对值
    auto abs_wz = xt::fabs(wz);                  // |wz|：角速度绝对值

    // 逐时间步检查：若瞬时转弯半径 r = |vx|/|wz| 小于机械下限，
    // 则将角速度削至 |wz| = |vx| / min_turning_r_（保持原方向，copysign 保号）
    for (size_t i = 0; i < wz.size(); ++i) {
      if ((abs_vx[i] / abs_wz[i]) < min_turning_r_) {
        wz[i] = std::copysign(abs_vx[i] / min_turning_r_, wz[i]);
      }
    }
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};  // 最小转弯半径（米），由参数配置
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 * （差速驱动运动模型：左右轮差速实现转向，无侧向速度，
 *  无额外硬约束，直接复用基类的一阶直通 predict()。）
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;  // 差速底盘不能侧向平移
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 * （全向运动模型：带侧向轮（如麦克纳姆轮），vy 可用。
 *  predict() 会额外拷贝 cvy -> vy，其余与差速模型一致。）
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;  // 全向底盘可以使用 Y 轴侧向速度
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

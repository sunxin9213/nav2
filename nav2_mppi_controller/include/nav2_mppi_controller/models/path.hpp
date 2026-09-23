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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Path
 * @brief Path represented as a tensor
 *（路径张量模型：将全局路径转换为便于向量化计算的一维张量）
 */
struct Path
{
  xt::xtensor<float, 1> x;     // 路径点的 x 坐标序列
  xt::xtensor<float, 1> y;     // 路径点的 y 坐标序列
  xt::xtensor<float, 1> yaws;  // 路径点的朝向角序列（rad）

  /**
    * @brief Reset path data
    *（重置路径数据：按路径点数量分配并清零）
    */
  void reset(unsigned int size)
  {
    x = xt::zeros<float>({size});
    y = xt::zeros<float>({size});
    yaws = xt::zeros<float>({size});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_

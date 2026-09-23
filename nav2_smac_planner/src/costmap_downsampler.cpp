// Copyright (c) 2020, Carlos Luis
// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#include "nav2_smac_planner/costmap_downsampler.hpp"

#include <string>
#include <memory>
#include <algorithm>

namespace nav2_smac_planner
{

// 【中文注释】CostmapDownsampler：代价地图下采样器。
// 用途：大地图上做全图搜索（如 SmacPlanner2D、Hybrid 的障碍启发）太慢，
// 按 downsampling_factor 把每 factor x factor 个栅格合并成一个粗栅格，
// 状态数按 factor^2 缩减。合并规则二选一：
//   use_min_cost_neighbor=false（默认）：取最大代价——保守，任何子格是障碍则粗格是障碍；
//   use_min_cost_neighbor=true：取最小代价——激进，用于障碍启发等允许低估的场景。
CostmapDownsampler::CostmapDownsampler()
: _costmap(nullptr),
  _downsampled_costmap(nullptr),
  _downsampled_costmap_pub(nullptr)
{
}

CostmapDownsampler::~CostmapDownsampler()
{
}

void CostmapDownsampler::on_configure(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & global_frame,
  const std::string & topic_name,
  nav2_costmap_2d::Costmap2D * const costmap,
  const unsigned int & downsampling_factor,
  const bool & use_min_cost_neighbor)
{
  // 【中文注释】配置阶段：记录原始地图、采样倍率和合并规则，
  // 计算下采样后尺寸（分辨率 = 原分辨率 x factor），分配粗粒度地图；
  // 可选创建调试发布器把下采样地图发到 topic 可视化。
  _costmap = costmap;
  _downsampling_factor = downsampling_factor;
  _use_min_cost_neighbor = use_min_cost_neighbor;
  updateCostmapSize();

  _downsampled_costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
    _downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
    _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

  if (!node.expired()) {
    _downsampled_costmap_pub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node, _downsampled_costmap.get(), global_frame, topic_name, false);
  }
}

void CostmapDownsampler::on_activate()
{
  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->on_activate();
  }
}

void CostmapDownsampler::on_deactivate()
{
  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->on_deactivate();
  }
}

void CostmapDownsampler::on_cleanup()
{
  _costmap = nullptr;
  _downsampled_costmap.reset();
  _downsampled_costmap_pub.reset();
}

nav2_costmap_2d::Costmap2D * CostmapDownsampler::downsample(
  const unsigned int & downsampling_factor)
{
  // 【中文注释】执行下采样：更新采样倍率与尺寸，尺寸变化时重设粗地图，
  // 然后对每个粗栅格计算合并代价，最后可选发布调试 topic，返回下采样地图指针。
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();

  // Adjust costmap size if needed
  if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
    _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
    _downsampled_costmap->getResolution() != _downsampled_resolution)
  {
    resizeCostmap();
  }

  // Assign costs
  // 【中文注释】逐个粗栅格计算合并后的代价值
  for (unsigned int i = 0; i < _downsampled_size_x; ++i) {
    for (unsigned int j = 0; j < _downsampled_size_y; ++j) {
      setCostOfCell(i, j);
    }
  }

  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->publishCostmap();
  }
  return _downsampled_costmap.get();
}

void CostmapDownsampler::updateCostmapSize()
{
  // 【中文注释】计算下采样后尺寸：ceil(原尺寸/factor)，分辨率放大 factor 倍
  _size_x = _costmap->getSizeInCellsX();
  _size_y = _costmap->getSizeInCellsY();
  _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
  _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
  _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
}

void CostmapDownsampler::resizeCostmap()
{
  // 【中文注释】原地图尺寸变化时（如 rolling window）重设粗地图大小
  _downsampled_costmap->resizeMap(
    _downsampled_size_x,
    _downsampled_size_y,
    _downsampled_resolution,
    _costmap->getOriginX(),
    _costmap->getOriginY());
}

void CostmapDownsampler::setCostOfCell(
  const unsigned int & new_mx,
  const unsigned int & new_my)
{
  // 【中文注释】计算单个粗栅格的合并代价：
  // 遍历它覆盖的 factor x factor 个原始栅格（越界跳过），
  // 取最大（保守）或最小（激进）代价值写入粗地图。
  // 注意初值：取最大时从 0 开始，取最小时从 255 开始。
  unsigned int mx, my;
  unsigned char cost = _use_min_cost_neighbor ? 255 : 0;
  unsigned int x_offset = new_mx * _downsampling_factor;
  unsigned int y_offset = new_my * _downsampling_factor;

  for (unsigned int i = 0; i < _downsampling_factor; ++i) {
    mx = x_offset + i;
    if (mx >= _size_x) {
      continue;
    }
    for (unsigned int j = 0; j < _downsampling_factor; ++j) {
      my = y_offset + j;
      if (my >= _size_y) {
        continue;
      }
      cost = _use_min_cost_neighbor ?
        std::min(cost, _costmap->getCost(mx, my)) :
        std::max(cost, _costmap->getCost(mx, my));
    }
  }

  _downsampled_costmap->setCost(new_mx, new_my, cost);
}

}  // namespace nav2_smac_planner

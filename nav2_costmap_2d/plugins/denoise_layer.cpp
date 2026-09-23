// Copyright (c) 2023 Andrey Ryzhikov
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
// ============================================================================
// 中文注释：去噪层（DenoiseLayer）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图插件体系中的"去噪层"。它不产生新障碍，
// 而是对主代价地图做图像形态学后处理：找出孤立的小块障碍（连通域）
// 并清除，以消除传感器噪声/短暂误检留下的"孤点障碍"。
//
// 【在 Nav2 中的位置】
// LayeredCostmap 分层体系中的后处理层，通常放在 layers 列表中
// 感知层（Obstacle/Voxel）之后、InflationLayer 之前，
// 避免噪声点被膨胀成大片高代价区域。
//
// 【核心概念】
// 1. 连通域：以 4 连通（上下左右）或 8 连通（含对角线）判断相邻障碍
//    是否属于同一组；小于 minimal_group_size 的组被视为噪声清除。
// 2. 两种算法：
//    - minimal_group_size=2：腐蚀式快速算法 removeSinglePixels()
//      （一次膨胀取邻域最大值，无邻居的孤立点被清除）；
//    - >2：通用连通域分割算法 removeGroups()。
// 3. Image 视图：用免拷贝的 ROI 图像包装主地图内存
//    （unsigned char 一维数组，行步长 step = size_x）。
// 4. 背景判定 isBackground：FREE_SPACE 等非障碍值是背景，
//    LETHAL_OBSTACLE / INSCRIBED_INFLATED_OBSTACLE（可选 NO_INFORMATION）
//    是前景（障碍）。
// ============================================================================

#include "nav2_costmap_2d/denoise_layer.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

// ============================================================================
// onInitialize()：读取去噪参数（组大小阈值、连通类型）
// ============================================================================
void
DenoiseLayer::onInitialize()
{
  // Enable/disable plugin
  // 是否启用本层
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smaller groups should be filtered
  // 小于该尺寸的障碍连通组将被视为噪声清除
  declareParameter("minimal_group_size", rclcpp::ParameterValue(2));
  // Pixels connectivity type
  // 连通类型：4（水平垂直相邻）或 8（含对角线）
  declareParameter("group_connectivity_type", rclcpp::ParameterValue(8));

  const auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("DenoiseLayer::onInitialize: Failed to lock node");
  }
  node->get_parameter(name_ + "." + "enabled", enabled_);

  // 辅助 lambda：按层前缀读取 int 参数
  auto getInt = [&](const std::string & parameter_name) {
      int param{};
      node->get_parameter(name_ + "." + parameter_name, param);
      return param;
    };

  const int minimal_group_size_param = getInt("minimal_group_size");

  // 阈值 <=1 意味着不去除任何像素（单像素也是"组"）
  if (minimal_group_size_param <= 1) {
    RCLCPP_WARN(
      logger_,
      "DenoiseLayer::onInitialize(): param minimal_group_size: %i."
      " A value of 1 or less means that all map cells will be left as they are.",
      minimal_group_size_param);
    minimal_group_size_ = 1;
  } else {
    minimal_group_size_ = static_cast<size_t>(minimal_group_size_param);
  }

  const int group_connectivity_type_param = getInt("group_connectivity_type");

  // 解析连通类型：4 或 8（默认 8）
  if (group_connectivity_type_param == 4) {
    group_connectivity_type_ = ConnectivityType::Way4;
  } else {
    group_connectivity_type_ = ConnectivityType::Way8;

    if (group_connectivity_type_param != 8) {
      RCLCPP_WARN(
        logger_, "DenoiseLayer::onInitialize(): param group_connectivity_type: %i."
        " Possible values are  4 (neighbors pixels are connected horizontally and vertically) "
        "or 8 (neighbors pixels are connected horizontally, vertically and diagonally)."
        "The default value 8 will be used",
        group_connectivity_type_param);
    }
  }

  current_ = true;
}

// reset()：复位层状态（本层无内部状态需清空，仅标记不最新）
void
DenoiseLayer::reset()
{
  current_ = false;
}

// isClearable()：本层不支持 clear costmap 服务的清除
bool
DenoiseLayer::isClearable()
{
  return false;
}

// ============================================================================
// updateBounds()：分层更新第一阶段——去噪层无边界扩展需求（空实现）
// 它只处理其他层已写入主地图的数据
// ============================================================================
void
DenoiseLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * /*min_x*/, double * /*min_y*/,
  double * /*max_x*/, double * /*max_y*/) {}

// ============================================================================
// updateCosts()：分层更新第二阶段——直接在主地图上做去噪
// （注意：与其他层不同，本层没有内部缓存，就地修改 master_grid）
// ============================================================================
void
DenoiseLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_x, int min_y, int max_x, int max_y)
{
  if (!enabled_) {
    return;
  }

  // 空区域直接返回
  if (min_x >= max_x || min_y >= max_y) {
    return;
  }
  // 判断 NO_INFORMATION(255) 是否视为障碍：
  // 主地图默认值不是 NO_INFORMATION 时，255 出现即当作障碍处理
  no_information_is_obstacle_ = master_grid.getDefaultValue() != NO_INFORMATION;

  // wrap roi_image over existing costmap2d buffer
  // 用 ROI 图像视图直接包装主地图缓冲区（零拷贝）：
  // 起始指针 = master_array + min_y*step + min_x，行步长 step = size_x
  unsigned char * master_array = master_grid.getCharMap();
  const int step = static_cast<int>(master_grid.getSizeInCellsX());

  const size_t width = max_x - min_x;
  const size_t height = max_y - min_y;
  Image<uint8_t> roi_image(height, width, master_array + min_y * step + min_x, step);

  try {
    denoise(roi_image);   // 执行去噪（就地修改主地图）
  } catch (std::exception & ex) {
    RCLCPP_ERROR(logger_, "%s", (std::string("Inner error: ") + ex.what()).c_str());
  }

  current_ = true;
}

// ============================================================================
// denoise()：去噪入口——按 minimal_group_size 选择算法
// =2 用快速腐蚀法；>2 用通用连通域分割法
// ============================================================================
void
DenoiseLayer::denoise(Image<uint8_t> & image) const
{
  if (image.empty()) {
    return;
  }

  if (minimal_group_size_ <= 1) {
    return;  // A smaller group cannot exist. No one pixel will be changed
    // 阈值<=1：不存在更小的组，任何像素都不会被修改
  }

  if (minimal_group_size_ == 2) {
    // Performs fast filtration based on erosion function
    // 快速路径：清除孤立单像素（腐蚀式算法）
    removeSinglePixels(image);
  } else {
    // Performs a slower segmentation-based operation
    // 通用路径：连通域分割后清除小团块
    removeGroups(image);
  }
}

// ============================================================================
// removeGroups()：通用算法——委托给 GroupsRemover 做连通域分割
// 以 isBackground 为前景判定谓词，移除小于阈值的组
// ============================================================================
void
DenoiseLayer::removeGroups(Image<uint8_t> & image) const
{
  groups_remover_.removeGroups(
    image, buffer_, group_connectivity_type_, minimal_group_size_,
    [this](uint8_t pixel) {return isBackground(pixel);});
}

// ============================================================================
// removeSinglePixels()：快速算法——形态学腐蚀清除孤立点
// 思路：对每个像素取其邻域（4/8 连通）的最大代价值，
// 若自身是障碍而邻域最大值是背景（无障碍邻居），则置为自由空间
// ============================================================================
void
DenoiseLayer::removeSinglePixels(Image<uint8_t> & image) const
{
  // Building a map of 4 or 8-connected neighbors.
  // The pixel of the map is 255 if there is an obstacle nearby
  // 邻域图缓冲区：每个像素记录其邻域中的最大代价值
  uint8_t * buf = buffer_.get<uint8_t>(image.rows() * image.columns());
  Image<uint8_t> max_neighbors_image(image.rows(), image.columns(), buf, image.columns());

  // If NO_INFORMATION (=255) isn't obstacle, we can't use a simple max() to check
  // any obstacle nearby. In this case, we interpret NO_INFORMATION as an empty space.
  // 注意：若 NO_INFORMATION 不算障碍，直接取 max 会被 255 误导，
  // 必须先把 255 替换成 FREE_SPACE 再取最大值
  if (!no_information_is_obstacle_) {
    // 替换函数：NO_INFORMATION → FREE_SPACE
    auto replace_to_free = [](uint8_t v) {
        return v == NO_INFORMATION ? FREE_SPACE : v;
      };
    // 三元素最大值（膨胀窗口需要 3 个输入像素）
    auto max = [&](const std::initializer_list<uint8_t> lst) {
        std::array<uint8_t, 3> buf = {
          replace_to_free(*lst.begin()),
          replace_to_free(*(lst.begin() + 1)),
          replace_to_free(*(lst.begin() + 2))
        };
        return *std::max_element(buf.begin(), buf.end());
      };
    // 膨胀操作（取邻域最大值）生成邻域图
    dilate(image, max_neighbors_image, group_connectivity_type_, max);
  } else {
    // NO_INFORMATION 视为障碍时可直接取 max
    auto max = [](const std::initializer_list<uint8_t> lst) {
        return std::max(lst);
      };
    dilate(image, max_neighbors_image, group_connectivity_type_, max);
  }

  // 腐蚀判定：自身是障碍(非背景)且邻域全为背景 → 孤立点，清除为自由空间
  max_neighbors_image.convert(
    image, [this](uint8_t maxNeighbor, uint8_t & img) {
      if (!isBackground(img) && isBackground(maxNeighbor)) {
        img = FREE_SPACE;
      }
    });
}

// ============================================================================
// isBackground()：背景（非障碍）判定
// 障碍 = LETHAL_OBSTACLE / INSCRIBED_INFLATED_OBSTACLE
//        （可选包含 NO_INFORMATION，取决于主地图默认值）
// ============================================================================
bool DenoiseLayer::isBackground(uint8_t pixel) const
{
  bool is_obstacle =
    pixel == LETHAL_OBSTACLE ||
    pixel == INSCRIBED_INFLATED_OBSTACLE ||
    (pixel == NO_INFORMATION && no_information_is_obstacle_);
  return !is_obstacle;
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a DenoiseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// pluginlib 导出宏：注册 DenoiseLayer 为 Layer 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DenoiseLayer, nav2_costmap_2d::Layer)

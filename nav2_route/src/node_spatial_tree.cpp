// Copyright (c) 2025, Open Navigation LLC
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

// ============================================================================
// 文件说明（中文）：
// NodeSpatialTree（节点空间树）的实现。
// 模块定位：路图节点的空间索引（R 树/KD 树等价物），基于 nanoflann
// 的 KD 树实现。用途：给定一个位姿（如机器人当前位置或目标点），
// 快速找出路图中距离最近的 N 个图节点，供 GoalIntentExtractor
// 确定"起终点对应哪个图节点"使用。相比暴力遍历 O(n)，
// KD 树近邻查询复杂度约 O(log n)。
// ============================================================================

#include <string>
#include <memory>
#include <vector>

#include "nav2_route/node_spatial_tree.hpp"

namespace nav2_route
{

// 析构：释放 KD 树与适配器（裸指针管理，需手动 delete）
NodeSpatialTree::~NodeSpatialTree()
{
  if (kdtree_) {
    delete kdtree_;
    kdtree_ = nullptr;
  }

  if (adaptor_) {
    delete adaptor_;
    adaptor_ = nullptr;
  }
}

// 设置近邻查询返回的节点数量 K
void NodeSpatialTree::setNumOfNearestNodes(int num_of_nearest_nodes)
{
  num_of_nearest_nodes_ = num_of_nearest_nodes;
}

// ---------------------------------------------------------------------------
// （重新）构建 KD 树：
//   - 释放旧的树与适配器；
//   - GraphAdaptor 把 nav2_route::Graph 包装成 nanoflann 需要的
//     数据集接口（提供节点数量与按维取坐标的方法）；
//   - 构建二维（DIMENSION=2）KD 树索引（参数 10 为最大叶深度）；
//   - 保存图指针供后续查询。
// ---------------------------------------------------------------------------
void NodeSpatialTree::computeTree(Graph & graph)
{
  if (kdtree_) {
    delete kdtree_;
    kdtree_ = nullptr;
  }

  if (adaptor_) {
    delete adaptor_;
    adaptor_ = nullptr;
  }

  adaptor_ = new GraphAdaptor(graph);
  kdtree_ = new kd_tree_t(DIMENSION, *adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree_->buildIndex();
  graph_ = &graph;
}

// ---------------------------------------------------------------------------
// K 近邻查询：找位姿附近最近的 num_of_nearest_nodes_ 个图节点。
// 返回值：查询到的节点下标（对应 graph 中的位置）追加到 node_ids；
// 无结果（空树）返回 false。
// ---------------------------------------------------------------------------
bool NodeSpatialTree::findNearestGraphNodesToPose(
  const geometry_msgs::msg::PoseStamped & pose_in, std::vector<unsigned int> & node_ids)
{
  size_t num_results = static_cast<size_t>(num_of_nearest_nodes_);
  std::vector<unsigned int> ret_index(num_results);   // 结果下标数组
  std::vector<double> out_dist_sqr(num_results);      // 对应距离平方（未使用）
  // 查询点：位姿的 (x, y)
  const double query_pt[2] = {pose_in.pose.position.x, pose_in.pose.position.y};
  // KNN 搜索：返回实际找到的数量（可能小于请求的 K）
  num_results = kdtree_->knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

  if (num_results == 0) {
    return false;
  }

  // 将结果下标输出给调用方
  for (int i = 0; i < static_cast<int>(ret_index.size()); ++i) {
    node_ids.push_back(ret_index[i]);
  }
  return true;
}

}  // namespace nav2_route

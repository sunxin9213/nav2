// Copyright (c) 2021, Samsung Research America
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

#include "nav2_smac_planner/node_basic.hpp"

namespace nav2_smac_planner
{

// 【中文注释】NodeBasic：优先队列中的轻量节点句柄。
// 为了让最小堆只存"索引+位姿快照"而非完整节点（拷贝便宜、缓存友好），
// 入队时(populateSearchNode)从图节点复制连续坐标位姿等搜索所需信息，
// 出队时(processSearchNode)把快照写回图节点——这样 Hybrid/Lattice 的
// 连续坐标位姿才能跟随"最后一次入队时找到它的那条基元"。

template<typename Node2D>
void NodeBasic<Node2D>::processSearchNode()
{
  // 【中文注释】Node2D 特化：节点状态完全由索引决定，无需恢复快照
}

template<>
void NodeBasic<NodeHybrid>::processSearchNode()
{
  // We only want to override the node's pose if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  // 【中文注释】Hybrid 版本：节点未被访问过时，把队列快照中的
  // 连续位姿和基元索引写回图节点（同一节点多次入队时，
  // 只有点位姿随最优分支更新，已关闭节点不被覆盖）
  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitiveIndex(this->motion_index);
  }
}

template<>
void NodeBasic<NodeLattice>::processSearchNode()
{
  // We only want to override the node's pose/primitive if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  // 【中文注释】Lattice 版本：同上，额外恢复运动基元指针和倒车标记
  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitive(this->prim_ptr);
    this->graph_node_ptr->backwards(this->backward);
  }
}

template<>
void NodeBasic<Node2D>::populateSearchNode(Node2D * & node)
{
  // 【中文注释】Node2D：只记录图节点指针
  this->graph_node_ptr = node;
}

template<>
void NodeBasic<NodeHybrid>::populateSearchNode(NodeHybrid * & node)
{
  // 【中文注释】Hybrid：入队时快照连续位姿和基元索引
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->motion_index = node->getMotionPrimitiveIndex();
}

template<>
void NodeBasic<NodeLattice>::populateSearchNode(NodeLattice * & node)
{
  // 【中文注释】Lattice：入队时快照位姿、基元指针和倒车标记
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->prim_ptr = node->getMotionPrimitive();
  this->backward = node->isBackward();
}

template class NodeBasic<Node2D>;
template class NodeBasic<NodeHybrid>;
template class NodeBasic<NodeLattice>;

}  // namespace nav2_smac_planner

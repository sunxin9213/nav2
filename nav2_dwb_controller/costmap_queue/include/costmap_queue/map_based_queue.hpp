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

#ifndef COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_
#define COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_

#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

// ============================================================================
// 模块说明：基于 map 的优先队列（MapBasedQueue）
// ----------------------------------------------------------------------------
// costmap_queue 包的底层数据结构，实现一个以 double 为优先级的模板化
// 优先队列。它是 std::priority_queue 的针对性优化替代品：
//
// 【为什么比 std::priority_queue 快？】
//   std::priority_queue 是全局二叉堆：任何元素的插入/弹出都可能触发
//   O(log n) 的堆调整，且弹出后需要重新排序。而 MapBasedQueue 的结构是：
//     std::map<double, std::vector<item_t>>
//   即"优先级 -> 同优先级元素桶"的有序映射 + 桶内 vector：
//   - 同优先级的元素入队/出队只是 vector 的 push_back / pop_back，O(1)；
//   - 出队后（pop）用 find_if 找到下一个"非空桶"，且得益于 map 的有序性
//     以及出队顺序近似递增的特性，该查找通常只前进一两步，接近 O(1)；
//   - 关键优势：迭代（逐个取出）过程不需要对整个队列重新排序。
//   在代价地图 BFS 场景中，优先级就是距离，大量格子共享相同距离值
//   （例如曼哈顿距离为整数），桶结构非常契合。
//
// 【性能相关参数 reset_bins】
//   若每次迭代插入的优先级模式相同（BFS 扩散正是如此），可以把
//   reset_bins 设为 false：reset() 时不再销毁各优先级桶（保留空桶结构），
//   避免下一轮迭代重建 map 节点的开销。
//
// 【迭代器缓存优化】
//   - last_insert_iter_：缓存最近一次插入的桶迭代器。连续插入相同
//     优先级的元素时（BFS 中极其常见）免去 map::find；
//   - iter_：当前出队位置的迭代器。新入队元素优先级更小时才回移。
// ============================================================================

namespace costmap_queue
{
/**
 * @brief Templatized interface for a priority queue
 *
 * This is faster than the std::priority_queue implementation in certain cases because iterating does
 * not require resorting after every element is examined.
 * Based on https://github.com/ros-planning/navigation/pull/525
 * The relative speed of this against the priority queue depends how many items with each
 * priority are inserted into the queue.
 *
 * One additional speed up depends on the patterns of priorities during each iteration of the queue.
 * If the same priorities are inserted into the queue on every iteration, then it is quicker to
 * set reset_bins = false, such that the priority bins are not reset and will not have to be recreated
 * on each iteration.
 */
template<class item_t>
class MapBasedQueue
{
public:
  /**
   * @brief Default Constructor
   */
  // 构造：reset_bins=true 表示每次 reset 都清空全部优先级桶；
  // false 则保留桶结构（复用 map 节点），适合周期性重复的优先级模式
  explicit MapBasedQueue(bool reset_bins = true)
  : reset_bins_(reset_bins), item_count_(0)
  {
    reset();
  }

  /**
   * @brief Clear the queue
   */
  // 清空队列：仅当 reset_bins_ 为真或队列非空时才真正销毁桶；
  // 两个迭代器（出队位置/最近插入位置）一律重置为 map 末尾（无效标记）
  virtual void reset()
  {
    if (reset_bins_ || item_count_ > 0) {
      item_bins_.clear();
      item_count_ = 0;
    }
    iter_ = last_insert_iter_ = item_bins_.end();
  }

  /**
   * @brief Add a new item to the queue with a set priority
   * @param priority Priority of the item
   * @param item Payload item
   */
  // 入队：将 item 放入指定优先级的桶
  void enqueue(const double priority, item_t item)
  {
    // We keep track of the last priority we inserted. If this items priority
    // matches the previous insertion we can avoid searching through all the
    // bins.
    // 优化：若与上一次插入的优先级相同（BFS 扩散常见），直接复用缓存迭代器
    if (last_insert_iter_ == item_bins_.end() || last_insert_iter_->first != priority) {
      // 优先级变化了，需要在 map 中查找对应的桶
      last_insert_iter_ = item_bins_.find(priority);

      // If not found, create a new bin
      // 没找到则创建新桶（新优先级第一次出现）
      if (last_insert_iter_ == item_bins_.end()) {
        auto map_item = std::make_pair(priority, std::move(std::vector<item_t>()));

        // Inserts an item if it doesn't exist. Returns an iterator to the item
        // whether it existed or was inserted.
        // map::insert：不存在则插入，返回指向该键值对的迭代器
        std::pair<ItemMapIterator, bool> insert_result = item_bins_.insert(std::move(map_item));
        last_insert_iter_ = insert_result.first;
      }
    }

    // Add the item to the vector for this map key
    // 元素压入该优先级桶的 vector 末尾（LIFO，出队时从尾部弹出）
    last_insert_iter_->second.push_back(item);
    item_count_++;

    // Use short circuiting to check if we want to update the iterator
    // 若新元素优先级比当前出队位置更小，出队迭代器前移，保证始终指向最小优先级桶
    if (iter_ == item_bins_.end() || priority < iter_->first) {
      iter_ = last_insert_iter_;
    }
  }

  /**
   * @brief Check to see if there is anything in the queue
   * @return True if there is nothing in the queue
   *
   * Must be called prior to front/pop.
   */
  // 判空：必须在 front()/pop() 之前调用
  bool isEmpty()
  {
    return item_count_ == 0;
  }

  /**
   * @brief Return the item at the front of the queue
   * @return The item at the front of the queue
   */
  // 取队首（最小优先级桶中最后压入的元素）：空队列抛 out_of_range 异常
  item_t & front()
  {
    if (iter_ == item_bins_.end()) {
      throw std::out_of_range("front() called on empty costmap_queue::MapBasedQueue!");
    }

    return iter_->second.back();
  }

  /**
   * @brief Remove (and destroy) the item at the front of the queue
   */
  // 弹出队首元素：从当前桶的 vector 尾部弹出；
  // 随后 find_if 向后（优先级递增方向）寻找下一个非空桶
  void pop()
  {
    if (iter_ != item_bins_.end() && !iter_->second.empty()) {
      iter_->second.pop_back();
      item_count_--;
    }

    // lambda：判断某桶是否非空
    auto not_empty = [](const typename ItemMap::value_type & key_val) {
        return !key_val.second.empty();
      };
    iter_ = std::find_if(iter_, item_bins_.end(), not_empty);
  }

protected:
  using ItemMap = std::map<double, std::vector<item_t>>;      // 优先级 -> 元素桶 的有序映射
  using ItemMapIterator = typename ItemMap::iterator;          // 桶迭代器类型

  bool reset_bins_;           // reset 时是否销毁全部桶

  ItemMap item_bins_;         // 队列主体：有序优先级桶集合
  unsigned int item_count_;   // 队列中元素总数（跨所有桶）
  ItemMapIterator iter_;      // 当前出队位置（最小非空桶）
  ItemMapIterator last_insert_iter_;  // 最近一次入队的桶（同优先级连续插入免查找）
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_

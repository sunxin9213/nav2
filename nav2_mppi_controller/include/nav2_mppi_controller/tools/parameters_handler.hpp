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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi
{
/**
 * @class Parameter Type enum
 */
// 参数类型：Dynamic 运行期可通过 ros2 param set 动态修改；
// Static 仅启动时读取，修改需重启节点
enum class ParameterType { Dynamic, Static };

/**
 * @class mppi::ParametersHandler
 * @brief Handles getting parameters and dynamic parmaeter changes
 * （MPPI 参数处理器：统一封装参数声明/读取/动态更新。
 *  典型用法：auto getParam = parameters_handler_->getParamGetter(name_);
 *  getParam(var_, "param_name", default); 命名空间自动拼接为 name_.param_name。
 *  动态参数通过 ROS2 add_on_set_parameters_callback 机制分发，
 *  配合 pre/post 回调保证优化器内部状态的一致性。）
 */
class ParametersHandler
{
public:
  // 参数更新回调签名：接收 rclcpp::Parameter（含名称与新值）
  using get_param_func_t = void (const rclcpp::Parameter & param);
  using post_callback_t = void ();   // 参数变更后的收尾回调签名
  using pre_callback_t = void ();   // 参数变更前的准备回调签名

  /**
    * @brief Constructor for mppi::ParametersHandler
    */
  ParametersHandler() = default;

  /**
    * @brief Constructor for mppi::ParametersHandler
    * @param parent Weak ptr to node
    */
  explicit ParametersHandler(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  /**
    * @brief Starts processing dynamic parameter changes
    */
  void start();  // 注册 ROS2 on_set_parameters 回调，启用动态调参

  /**
    * @brief Dynamic parameter callback
    * @param parameter Parameter changes to process
    * @return Set Parameter Result
    */
  // ROS2 参数变更回调实现（见 parameters_handler.cpp）
  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
    * @brief Get an object to retreive parameters
    * @param ns Namespace to get parameters within
    * @return Parameter getter object
    */
  // 返回绑定命名空间的读取器 lambda：后续调用自动加 "ns." 前缀
  inline auto getParamGetter(const std::string & ns);

  /**
    * @brief Set a callback to process after parameter changes
    * @param callback Callback function
    */
  template<typename T>
  void addPostCallback(T && callback);

  /**
    * @brief Set a callback to process before parameter changes
    * @param callback Callback function
    */
  template<typename T>
  void addPreCallback(T && callback);

  /**
    * @brief Set a parameter to a dynamic parameter callback
    * @param setting Parameter
    * @param name Name of parameter
    */
  // 为成员变量 setting 注册动态更新回调（改动实时写入该变量）
  template<typename T>
  void setDynamicParamCallback(T & setting, const std::string & name);

  /**
   * @brief Get mutex lock for changing parameters
   * @return Pointer to mutex
   */
  // 暴露互斥锁给外部（如优化器在打分循环中持锁，防止改参竞态）
  std::mutex * getLock()
  {
    return &parameters_change_mutex_;
  }

  /**
    * @brief Set a parameter to a dynamic parameter callback
    * @param name Name of parameter
    * @param callback Parameter callback
    */
  template<typename T>
  void addDynamicParamCallback(const std::string & name, T && callback);

protected:
  /**
    * @brief Gets parameter
    * @param setting Return Parameter type
    * @param name Parameter name
    * @param default_value Default parameter value
    * @param param_type Type of parameter (dynamic or static)
    */
  // 声明（若未声明）并读取参数；Dynamic 类型额外注册更新回调
  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, const std::string & name, ParamT default_value,
    ParameterType param_type = ParameterType::Dynamic);

  /**
    * @brief Set a parameter
    * @param setting Return Parameter type
    * @param name Parameter name
    * @param node Node to set parameter via
    */
  // 从节点读取参数值并做类型转换后写入 setting
  template<typename ParamT, typename SettingT, typename NodeT>
  void setParam(SettingT & setting, const std::string & name, NodeT node) const;

  /**
    * @brief Converts parameter type to real types
    * @param parameter Parameter to convert into real type
    * @return parameter as a functional type
    */
  // 编译期按 T 的类型选择 rclcpp::Parameter 的 as_xxx() 取值接口
  template<typename T>
  static auto as(const rclcpp::Parameter & parameter);

  std::mutex parameters_change_mutex_;  // 保护回调表与参数变更的互斥锁
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_param_handler_;               // ROS2 参数回调句柄（cleanup 时注销）
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  // 生命周期节点弱引用
  std::string node_name_;               // 节点名（顶层命名空间）

  bool verbose_{false};                 // 是否打印参数日志

  // 参数名 -> 更新回调 的映射表，动态变更时按名分发
  std::unordered_map<std::string, std::function<get_param_func_t>>
    get_param_callbacks_;

  std::vector<std::function<pre_callback_t>> pre_callbacks_;   // 变更前回调列表
  std::vector<std::function<post_callback_t>> post_callbacks_;  // 变更后回调列表
};

// 生成绑定命名空间的参数读取 lambda：把 "param" 补全为 "ns.param"
inline auto ParametersHandler::getParamGetter(const std::string & ns)
{
  return [this, ns](
    auto & setting, const std::string & name, auto default_value,
    ParameterType param_type = ParameterType::Dynamic) {
           getParam(
             setting, ns.empty() ? name : ns + "." + name,
             std::move(default_value), param_type);
         };
}

template<typename T>
void ParametersHandler::addDynamicParamCallback(const std::string & name, T && callback)
{
  get_param_callbacks_[name] = callback;  // 注册/覆盖某参数的更新回调
}

template<typename T>
void ParametersHandler::addPostCallback(T && callback)
{
  post_callbacks_.push_back(callback);  // 追加变更后回调
}

template<typename T>
void ParametersHandler::addPreCallback(T && callback)
{
  pre_callbacks_.push_back(callback);  // 追加变更前回调
}

template<typename SettingT, typename ParamT>
void ParametersHandler::getParam(
  SettingT & setting, const std::string & name,
  ParamT default_value,
  ParameterType param_type)
{
  auto node = node_.lock();

  // 若节点尚未声明该参数则用默认值声明（幂等，避免重复声明报错）
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));

  // 读取当前值写入 setting
  setParam<ParamT>(setting, name, node);

  // 动态参数：注册运行期更新回调
  if (param_type == ParameterType::Dynamic) {
    setDynamicParamCallback(setting, name);
  }
}

template<typename ParamT, typename SettingT, typename NodeT>
void ParametersHandler::setParam(
  SettingT & setting, const std::string & name, NodeT node) const
{
  ParamT param_in{};
  node->get_parameter(name, param_in);          // 按 ROS 参数类型读取
  setting = static_cast<SettingT>(param_in);    // 转换为成员变量类型
}

template<typename T>
void ParametersHandler::setDynamicParamCallback(T & setting, const std::string & name)
{
  // 幂等：已注册过同名参数则跳过
  if (get_param_callbacks_.find(name) != get_param_callbacks_.end()) {
    return;
  }

  // lambda 捕获成员变量引用：参数变更时直接写入 setting（原地更新）
  auto callback = [this, &setting, name](const rclcpp::Parameter & param) {
      setting = as<T>(param);

      if (verbose_) {
        RCLCPP_INFO(logger_, "Dynamic parameter changed: %s", std::to_string(param).c_str());
      }
    };

  addDynamicParamCallback(name, callback);

  if (verbose_) {
    RCLCPP_INFO(logger_, "Dynamic Parameter added %s", name.c_str());
  }
}

template<typename T>
auto ParametersHandler::as(const rclcpp::Parameter & parameter)
{
  // 编译期分支（if constexpr）：按目标类型 T 选择正确的取值接口。
  // ROS2 参数只能存储有限几种类型，这里一一映射
  if constexpr (std::is_same_v<T, bool>) {
    return parameter.as_bool();
  } else if constexpr (std::is_integral_v<T>) {
    return parameter.as_int();
  } else if constexpr (std::is_floating_point_v<T>) {
    return parameter.as_double();
  } else if constexpr (std::is_same_v<T, std::string>) {
    return parameter.as_string();
  } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
    return parameter.as_integer_array();
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    return parameter.as_double_array();
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return parameter.as_string_array();
  } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
    return parameter.as_bool_array();
  }
}

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_

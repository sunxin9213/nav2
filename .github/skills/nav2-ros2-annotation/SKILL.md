---
name: nav2-ros2-annotation
description: "Use when adding Chinese explanatory comments to Nav2 C++ headers and source files, especially nav2_behaviors or nav2_behavior_tree, including ROS 2 lifecycle nodes, parameters, actions, publishers, subscriptions, TF, pluginlib, callback loops, and behavior-tree plugins."
argument-hint: Specify the Nav2 package or directory to annotate.
---

# Nav2 ROS 2 注释工作流

为 Nav2 的 C++、Python launch、YAML 参数和 XML/URDF/SDF 工程文件添加面向学习和维护的中文注释，同时保留原有代码和英文注释。重点解释函数行为、关键命令、ROS 2 机制、参数含义和实际应用场景。

## 目标

- 不删除、不重排、不改写业务代码。
- 不删除原有英文注释、许可证头和 Doxygen 文档。
- 在真实的函数声明/定义、构造函数、析构函数、注册宏和关键逻辑块附近添加中文说明。
- 注释应解释“为什么这样做、输入是什么、状态如何变化、失败时会怎样”，而不是只翻译函数名。
- 对 ROS 2 API 给出足够的背景，使读者能理解其生命周期、通信模型和运行时行为。
- 对 YAML、XML、URDF、SDF 中的参数和标签解释默认值、单位、坐标系、依赖关系、运行影响及典型应用场景。
- 同一种常见命令或 API 只做一次完整解释；其他位置使用简短提示，避免重复堆叠。

## 处理范围

1. 找到目标包中的 `.cpp`、`.h`、`.hpp` 文件，并确认包的构建入口、插件描述文件和测试目录。
2. 先阅读拥有行为的核心抽象，再处理具体插件。例如：
   - `LifecycleNode` 及 `on_configure`、`on_activate`、`on_deactivate`、`on_cleanup`、`on_shutdown`。
   - `SimpleActionServer`、Action goal/result/feedback、取消和终止流程。
   - `create_publisher`、Lifecycle Publisher、QoS 深度和发布时机。
   - `declare_parameter`、`ParameterValue`、`get_parameter` 和默认值。
   - `tf2_ros::Buffer`、TransformListener、坐标系和 transform tolerance。
   - `pluginlib::ClassLoader`、插件类型参数和动态实例化。
   - Costmap、footprint、collision checker 和控制循环。
3. 按文件逐步编辑；先完成核心基类/服务端，再完成插件，最后处理测试和辅助头文件。
4. 对 launch/YAML/XML 资源先确认结构，再在节点、参数组、元素或插件标签附近添加说明；不能把解释文字写入参数值、字典或 XML 属性。

## 注释粒度

### 函数注释

函数注释放在函数签名前一行，至少说明：

- 触发时机或调用者。
- 读取的输入、参数或 ROS 2 消息。
- 主要计算、状态转换或资源操作。
- 返回值的语义，以及成功、失败、取消或超时时的结果。

构造函数应说明默认状态和依赖初始化；析构函数应说明释放的资源。虚函数应说明基类契约以及派生类需要满足的条件。

### 关键逻辑块

只为有学习价值的逻辑块添加注释，例如：

- 参数声明和读取。
- 生命周期资源的创建、激活、停用和清理。
- Action Server 的 goal、cancel、feedback、result 流程。
- TF 查询和坐标变换容差判断。
- 控制循环的频率、超时、速度限制和停止逻辑。
- 碰撞预测、路径有效性、重试和异常处理。
- pluginlib 注册、加载、类型解析和失败回滚。

不要在普通函数调用、变量赋值、`if` 条件或每一条重复命令前机械插入注释。若一段命令已在第一次出现处解释，后续只保留必要的局部语义说明。

### YAML、XML 和仿真资源

- YAML 注释放在对应键或参数组前，不改变缩进、键名、列表和标量值。
- XML/URDF/SDF 使用合法的 `<!-- ... -->` 注释，说明元素层级、坐标单位、传感器频率、质量/惯量、插件文件和 ROS 2 topic/frame 映射。
- `origin/pose` 明确位置单位为米、姿态单位为弧度；速度、加速度、分辨率、阈值和频率按实际定义说明。
- launch 中的 `DeclareLaunchArgument`、`LaunchConfiguration`、`RewrittenYaml`、namespace、条件分支和 composable node 要解释求值时机和应用场景。

## ROS 2 解释规范

第一次遇到相关 API 时，使用“机制 + 当前代码作用 + 参数值含义 + 应用场景”的结构。例如：

- `LifecycleNode`：解释节点为何分阶段配置、激活和清理，以及 inactive 状态下为何不应发布控制命令。
- `declare_parameter("cycle_frequency", 10.0)`：解释参数名、默认值的单位/含义、提高或降低数值的影响，以及适合控制循环的场景。
- `create_publisher<T>("cmd_vel", 1)`：解释 topic 类型、名称、QoS 深度 1 的取舍，以及速度命令为什么通常需要及时性。
- `SimpleActionServer`：解释 goal、feedback、result、cancel 的异步通信关系和当前行为的执行线程/循环。
- `RCLCPP_INFO/WARN/ERROR`：第一次出现时说明日志级别和使用场景，后续无需重复定义。
- `tf2_ros::Buffer` 与 `TransformListener`：解释 TF 数据来源、坐标系关系、时间戳和 `transform_tolerance` 对传感器延迟的影响。
- `pluginlib::ClassLoader`：解释插件描述、类名字符串、动态加载和配置失败时的处理。

参数解释要明确单位和约束：频率通常是 Hz，时间通常是秒，线速度是 m/s，角速度是 rad/s，角加速度是 rad/s²，距离是 m，角度是 rad；如果代码或消息定义使用其他单位，以实际定义为准。不要臆测未能从代码、消息定义或配置文件确认的单位。

## 注释复用规则

- 为每个重复出现的 ROS 2 概念建立一次完整解释，优先放在基类、服务端或第一次出现的实现处。
- 后续相同 API 只解释本处的特殊参数或业务用途。
- 不复制整段通用说明，不生成大量“实现该逻辑”之类没有信息量的句子。
- 对相同参数名保持术语和单位一致；如果不同包的含义不同，明确指出局部差异。

## 执行步骤

1. **建立局部假设**：先找到控制行为的核心类和一个相邻实现，明确最小修改面；确定一个可快速验证的检查，例如搜索误插入到函数体的注释。
2. **读取上下文**：查看目标包的头文件、核心 `.cpp`、插件 XML、参数文件和相邻测试。优先理解调用关系，不做无关的全仓库扫描。
3. **设计注释词汇表**：为生命周期、Action、参数、TF、pluginlib、QoS、日志等重复概念确定统一中文术语和首次解释位置。
4. **分批编辑**：每批处理少量相关文件，注释紧贴函数签名或关键逻辑块，保留原英文内容和代码格式。
5. **立即验证**：编辑后先做窄范围检查，确认编译文本未被改变、注释没有进入普通调用行、中文注释没有重复。随后再处理下一批文件。
6. **全包复核**：检查所有目标扩展名，确认核心函数、插件注册和 ROS 2 生命周期均有说明；搜索残留的低信息量模板句和重复注释。
7. **最终报告**：说明处理的目录、文件数量、主要 ROS 2 主题、保留英文注释的情况，以及执行过的验证命令。文档任务不应为了注释而修改运行时逻辑。

## 质量检查

完成前至少满足以下条件：

- 原代码、英文注释、许可证头和 Doxygen 注释仍存在。
- YAML/XML/URDF/SDF 注释保持格式合法，解析后参数键和值与修改前一致。
- 中文注释位于函数签名、声明或关键逻辑块附近，而不是普通调用和赋值之前。
- 构造、生命周期回调、Action 回调、插件注册和主要控制循环均有语义说明。
- 关键参数包含默认值、单位/范围、运行影响和适用场景；无法确认的内容明确标注为待查证。
- 高频 API 的完整解释只出现一次，后续没有机械复制。
- 没有重复的中文注释、连续堆叠的同义注释或“实现该逻辑”等空泛注释。
- 注释使用 UTF-8，代码文件仍可被现有构建系统读取。
- 对文档-only 修改至少执行文本级验证；若环境允许，执行目标包的格式检查、编译或相关测试，并区分已有失败与本次变更造成的失败。

## 推荐触发示例

- “用 Nav2 ROS 2 注释技能注释 `nav2_behaviors`，详细解释生命周期、Action Server 和参数默认值。”
- “继续注释这个 Nav2 包，保留英文注释，重复出现的 ROS 2 API 只在第一次详细说明。”
- “检查这个包里的中文注释是否误插入了普通函数调用，并补充 TF、QoS 和插件加载的应用场景。”

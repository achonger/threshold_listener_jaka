# threshold_listener_jaka

`threshold_listener_jaka` 是一个简单的 ROS（catkin）示例包，用于订阅 `/threshold_detect` 话题（`std_msgs/Int32`）并在终端输出检测结果。

## 项目功能

该包包含一个 C++ 节点：

- 节点名：`threshold_listener_node`
- 订阅话题：`/threshold_detect`
- 消息类型：`std_msgs/Int32`
- 回调逻辑：
  - 每次接收到消息都会打印数值；
  - 当数值等于 `1` 时，额外打印 `threshold detected`。

## 目录结构

```text
threshold_listener_jaka/
├── CMakeLists.txt
├── package.xml
└── src/
    └── threshold_listener_jaka_node.cpp
```

## 依赖环境

- ROS 1（支持 catkin 工作流，例如 Kinetic/Melodic/Noetic）
- `roscpp`
- `std_msgs`

> 这些依赖已经在 `package.xml` 和 `CMakeLists.txt` 中声明。

## 编译方法（catkin）

在你的 catkin 工作空间中执行：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果该包还未放入工作空间，请先将仓库放到 `~/catkin_ws/src/` 下再编译。

## 运行方法

### 1) 启动 ROS Master

```bash
roscore
```

### 2) 启动监听节点

新开一个终端并执行：

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun threshold_listener_jaka threshold_listener_jaka_node
```

看到类似日志表示节点已启动：

```text
threshold_listener_node started, waiting for /threshold_detect ...
```

### 3) 发布测试消息

再开一个终端，执行：

```bash
rostopic pub /threshold_detect std_msgs/Int32 "data: 1" -r 1
```

节点端会输出：

- `Received /threshold_detect: 1`
- `threshold detected`

如果发布其他值（如 `0` 或 `2`），只会打印收到的数值，不会打印 `threshold detected`。

## 代码说明

核心逻辑位于 `src/threshold_listener_jaka_node.cpp`：

1. 在 `main` 中初始化 ROS 节点并创建订阅器；
2. 使用 `thresholdCallback` 处理接收到的 `Int32` 消息；
3. 进入 `ros::spin()` 循环持续监听。

## 当前状态与可改进项

目前该项目是一个最小可运行示例，`package.xml` 中仍有模板占位信息（如版本 `0.0.0`、许可证 `TODO`）。如果计划长期维护，建议：

- 补充真实版本号与许可证；
- 添加 launch 文件（如 `launch/threshold_listener.launch`）；
- 增加参数化配置（例如可配置订阅话题名、触发阈值）；
- 增加测试代码与 CI 配置。

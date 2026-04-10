# threshold_listener_jaka

## 项目介绍

`threshold_listener_jaka` 用于多 JAKA 机械臂开环控制测试，支持：

- 通过 `multi_jaka_openloop.launch` 按参数启用 `jaka1~jaka4` 中任意子集；
- 每台机械臂在独立命名空间下启动 `driver + state_adapter + robot_state_publisher`；
- 可选启动 `openloop_move_jaka4` 示例节点，执行分步直线开环动作。

## 依赖环境

- Ubuntu 20.04 + ROS1 Noetic
- `roscpp`
- `std_msgs`
- `geometry_msgs`
- 运行时相关包（由你的工作空间提供）：
  - `jaka_driver`
  - `jaka_description`
  - （若你的驱动链路需要）`jaka_msgs`
  - `robot_state_publisher`

## 编译方法

```bash
cd ~/code/catkin_ws
catkin build threshold_listener_jaka
```

## Launch 参数说明（核心）

- `enable_jaka1~enable_jaka4`：控制是否启动对应机械臂（默认只启 `jaka1`）
- `jaka1_ip~jaka4_ip`：每台机械臂 IP
- `start_jaka4_demo`：是否在 launch 内同时启动 `openloop_move_jaka4`
- `urdf_file`：机械臂 URDF 文件，默认：

```xml
<arg name="urdf_file" default="$(find jaka_description)/urdf/jaka_zu3.urdf"/>
```

> 可通过命令行覆盖该参数，以支持其他机械臂型号。

## 一键启动示例

### 1) 单独启动 JAKA4 并执行开环动作

```bash
source ~/code/catkin_ws/devel/setup.bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=false enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  start_jaka4_demo:=true \
  urdf_file:=$(find jaka_description)/urdf/jaka_zu3.urdf
```

### 2) 启动多个机械臂（JAKA1 + JAKA4）

```bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=true enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  start_jaka4_demo:=false \
  urdf_file:=$(find jaka_description)/urdf/jaka_zu3.urdf
```

## openloop_move_jaka4 节点说明

- 默认控制命名空间 `jaka4`，但已支持参数化：
  - `~arm_ns`（默认 `jaka4`）
  - `~tool_pose_topic`（默认 `/${arm_ns}/tool_position`）
  - `~linear_move_topic`（默认 `/${arm_ns}/linear_move`）
- 动作逻辑保持不变：沿 X 负方向总计 100 mm，10 步，每步 10 mm，每步停留 10 秒。

## 可移植性说明

- 不使用磁盘绝对路径（如 `/home/...`）；
- 资源路径统一采用 ROS 标准查找方式（`$(find <package>)/...`）；
- 机械臂启停、URDF、驱动节点均可通过 launch 参数覆盖，便于不同机器/不同型号复用。

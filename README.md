# threshold_listener_jaka

## 项目介绍

`threshold_listener_jaka` 是一个 ROS1 包，提供：

- 多机械臂开环启动文件 `multi_jaka_openloop.launch`（支持 `jaka1~jaka4` 按参数启停）；
- 开环示例节点 `openloop_move_jaka4`（默认控制 `jaka4`，沿 X 负方向分步移动）；
- 包内自包含 ZU3 模型文件 `urdf/jaka_zu3.urdf`。

## 依赖

- Ubuntu 20.04
- ROS1 Noetic
- `roscpp`
- `std_msgs`
- `geometry_msgs`
- 运行时依赖（按你的工作空间实际安装）：
  - `jaka_driver`
  - `robot_state_publisher`

## 编译（与项目当前 CMake/catkin 配置一致）

> 本包是标准 catkin C++ 包，默认用 `catkin_make` 可直接编译。

```bash
cd ~/code/catkin_ws
catkin_make --pkg threshold_listener_jaka
source devel/setup.bash
```

如果你使用 `catkin_tools`，也可用：

```bash
cd ~/code/catkin_ws
catkin build threshold_listener_jaka
source devel/setup.bash
```

## 启动参数说明（核心）

- `enable_jaka1~enable_jaka4`：是否启动对应机械臂
- `jaka1_ip~jaka4_ip`：机械臂 IP
- `start_jaka4_demo`：是否在 launch 中同时启动 `openloop_move_jaka4`
- `urdf_file`：URDF 文件路径（默认使用当前包内 URDF）

当前默认值（与项目文件一致）：

```xml
<arg name="urdf_file" default="$(find threshold_listener_jaka)/urdf/jaka_zu3.urdf"/>
```

## 一键启动示例（与当前 launch 一致）

### 1) 单独启动 JAKA4 + 自动运行开环示例

```bash
source ~/code/catkin_ws/devel/setup.bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=false enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  start_jaka4_demo:=true \
  urdf_file:=$(find threshold_listener_jaka)/urdf/jaka_zu3.urdf
```

### 2) 启动多机械臂（JAKA1 + JAKA4，不自动运行示例）

```bash
source ~/code/catkin_ws/devel/setup.bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=true enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  start_jaka4_demo:=false \
  urdf_file:=$(find threshold_listener_jaka)/urdf/jaka_zu3.urdf
```

## 单独运行开环节点

```bash
source ~/code/catkin_ws/devel/setup.bash
rosrun threshold_listener_jaka openloop_move_jaka4
```

可选参数示例（切换命名空间）：

```bash
rosrun threshold_listener_jaka openloop_move_jaka4 _arm_ns:=jaka2
```

## 说明

- 为避免路径耦合，启动命令统一使用 `$(find threshold_listener_jaka)/...` 引用本包资源；
- 多臂开关通过 `enable_jakaX` 控制，互不影响；
- `urdf_file` 可覆盖，后续支持其他机型时无需改代码。

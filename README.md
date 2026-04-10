# threshold_listener_jaka

## 1. 项目说明

本项目用于 ROS1 Noetic 环境下的 JAKA 开环测试，当前重点是多机械臂开环启动与 `jaka4` 示例动作。

项目内已内置模型资源：

- `urdf/jaka_zu3.urdf`
- `meshes/jaka_zu3_meshes/*.STL`

因此本项目 **不再依赖外部 `jaka_description` 包来提供 ZU3 模型文件**。

---

## 2. 目录与关键文件

- `launch/multi_jaka_openloop.launch`：多臂开环启动文件
- `src/openloop_move_jaka4.cpp`：开环分步移动示例节点
- `scripts/start_jaka4_openloop.sh`：一键启动脚本（推荐）
- `urdf/jaka_zu3.urdf`：ZU3 机器人模型
- `meshes/jaka_zu3_meshes/`：模型网格目录

---

## 3. 依赖环境

- Ubuntu 20.04
- ROS1 Noetic
- catkin 工作空间：`/home/hanmo/code/catkin_ws`
- 运行时依赖包（按你的环境提供）：
  - `jaka_driver`
  - `robot_state_publisher`

---

## 4. 编译步骤（使用 catkin build）

```bash
cd /home/hanmo/code/catkin_ws
catkin build threshold_listener_jaka
source /home/hanmo/code/catkin_ws/devel/setup.bash
```

---

## 5. 启动前检查（建议先执行）

```bash
rospack find threshold_listener_jaka
ls /home/hanmo/code/catkin_ws/src/threshold_listener_jaka/urdf/jaka_zu3.urdf
ls /home/hanmo/code/catkin_ws/src/threshold_listener_jaka/meshes/jaka_zu3_meshes
```

---

## 6. 一键启动（推荐）

推荐使用脚本：

> 脚本会显式传入 `jaka4_ip`，并使用绝对路径传入 `urdf_file`。

```bash
bash /home/hanmo/code/catkin_ws/src/threshold_listener_jaka/scripts/start_jaka4_openloop.sh
```

### 为什么推荐脚本

在 shell 里直接写 `urdf_file:=$(find threshold_listener_jaka)/...` 时，`$(...)` 会被 Bash 当作命令替换，
容易误触系统 `find` 命令解析。脚本内部通过 `rospack find` 拿绝对路径，更稳妥。

脚本逻辑：

1. `source /home/hanmo/code/catkin_ws/devel/setup.bash`
2. `rospack find threshold_listener_jaka` 获取包绝对路径
3. 拼接 URDF 绝对路径并检查文件是否存在
4. 用绝对路径参数调用 launch

---

## 7. 手动启动（使用绝对路径）

```bash
source /home/hanmo/code/catkin_ws/devel/setup.bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=false enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  jaka4_ip:=192.168.1.103 \
  start_jaka4_demo:=true \
  urdf_file:=/home/hanmo/code/catkin_ws/src/threshold_listener_jaka/urdf/jaka_zu3.urdf
```

---

## 8. 已知问题与说明

1. 某些版本的 `jaka_driver` 不包含 `jaka_state_adapter_node` 可执行文件。  
   因此当前 launch 已避免强依赖该节点，防止因找不到可执行文件导致整套启动失败。

2. 若 `openloop_move_jaka4` 一直等待 `/jaka4/tool_position`，需要继续确认：  
   - 当前 `jaka_driver` 实际发布的话题名；  
   - 是否与本项目默认话题一致（默认 `/jaka4/tool_position` 与 `/jaka4/linear_move`）。

---

## 9. 关键 launch 参数

- `enable_jaka1~enable_jaka4`：分别控制 4 台机械臂是否启动
- `jaka1_ip~jaka4_ip`：各机械臂 IP
- `start_jaka4_demo`：是否同时启动 `openloop_move_jaka4`
- `urdf_file`：URDF 文件路径（默认值如下）

```xml
<arg name="urdf_file" default="$(find threshold_listener_jaka)/urdf/jaka_zu3.urdf"/>
```

## 10. 驱动 IP 参数名对齐说明

`jaka_driver` 实际读取的参数名是 `ip`（不是 `robot_ip`）。

因此本项目 `multi_jaka_openloop.launch` 已按 `ip` 传参，例如：

```xml
<param name="ip" value="$(arg jaka4_ip)"/>
```



## 11. 连接方式对齐说明

`multi_jaka_openloop.launch` 已按成功项目的连接方式对齐：

- group 级参数：`ip` + `robot_description`；
- driver 节点内同时传 `ip` 与 `robot_ip`；
- driver 节点内加入常用控制/状态话题 remap；
- 状态适配节点使用 `pkg="jaka_close_contro" type="jaka_state_adapter_node"`；
- `robot_state_publisher` 使用 `publish_frequency=100.0` 并 remap `/joint_states -> joint_states`。

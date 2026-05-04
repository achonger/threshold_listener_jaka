# threshold_listener_jaka

## 1. 项目说明

本项目用于 ROS1 Noetic 环境下的 JAKA 开环测试，当前重点是多机械臂开环启动与 `jaka4` **连续直线运动**示例。

项目内已内置模型资源：

- `urdf/jaka_zu3.urdf`
- `meshes/jaka_zu3_meshes/*.STL`

因此本项目 **不再依赖外部 `jaka_description` 包来提供 ZU3 模型文件**。

---

## 2. 目录与关键文件

- `launch/multi_jaka_openloop.launch`：多臂开环启动文件
- `src/openloop_move_jaka4.cpp`：`jaka4` 连续直线运动节点（单次 linear_move）
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

## 6. 一键启动当前 JAKA4 连续直线运动（推荐）

### 6.1 首次使用（给脚本执行权限）

```bash
chmod +x /home/hanmo/code/catkin_ws/src/threshold_listener_jaka/scripts/start_jaka4_openloop.sh
```

### 6.2 直接一键启动

> 脚本默认仅启动 `jaka4`，并通过 launch 传入连续直线运动参数（80mm，沿 X 负方向）：
> - `jaka4_ip:=192.168.1.103`
> - `start_jaka4_demo:=true`
> - `urdf_file` 绝对路径（自动解析）

```bash
/home/hanmo/code/catkin_ws/src/threshold_listener_jaka/scripts/start_jaka4_openloop.sh
```

### 6.3 如果你的机械臂 IP 不是 `192.168.1.103`

请编辑脚本中的 `jaka4_ip:=192.168.1.103` 为你的实际 IP，然后重新执行上一条命令。

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

## 8. 当前版本运动逻辑（请重点关注）

`openloop_move_jaka4` 当前版本是**连续直线运动模式**，不是多步 dwell 扫描：

1. 启动后读取当前 `/jaka4/jaka_driver/tool_position` 作为 `initial_pose`
2. 读取方向向量 `direction_x/y/z` 并归一化
3. 按 `line_distance_mm`（默认 80mm）计算目标点
4. 保持初始姿态不变，只改位置
5. 只调用一次 `/jaka4/jaka_driver/linear_move`
6. 运动监控期间持续监听 `/threshold_detect`
7. 若收到 `data==1`，调用 `/jaka4/jaka_driver/stop_move` 并退出
8. 未触发阈值则到预计时间后打印最终位姿并退出

---

## 9. 如何指定直线运动方向（direction_x/y/z）

当前 launch 已传默认方向：

- `direction_x=-1.0`
- `direction_y=0.0`
- `direction_z=0.0`
- `direction_frame=base`

表示沿 base 坐标系 X 负方向直线运动。

### 9.1 方向向量规则

- 方向向量可任意非零，节点内部会自动归一化；
- 若向量长度接近 0（`<1e-9`），节点会报错退出；
- `direction_frame=base`：按 base 坐标系解释；
- `direction_frame=tool`：按当前 TCP 姿态旋转到 base 后再执行。

### 9.2 示例：改成沿 +Y 方向运动

```bash
source /home/hanmo/code/catkin_ws/devel/setup.bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=false enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  jaka4_ip:=192.168.1.103 start_jaka4_demo:=true \
  direction_x:=0.0 direction_y:=1.0 direction_z:=0.0 direction_frame:=base
```

---

## 10. 如何设置运动距离和速度

当前代码/launch使用以下参数控制连续直线运动：

- `line_distance_mm`：直线距离（默认 `80.0`）
- `line_speed_mm_s`：线速度（默认 `5.0`）
- `line_acc_mm_s2`：线加速度（默认 `20.0`）

### 10.1 示例：改成 120mm、速度 8mm/s

```bash
source /home/hanmo/code/catkin_ws/devel/setup.bash
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=false enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  jaka4_ip:=192.168.1.103 start_jaka4_demo:=true \
  line_distance_mm:=120.0 line_speed_mm_s:=8.0 line_acc_mm_s2:=25.0
```

---

## 11. 如何通过 /threshold_detect 停止运动

节点订阅 `std_msgs/Int32` 类型的 `/threshold_detect`：

- 收到 `data==1`：触发停止，调用 `/jaka4/jaka_driver/stop_move`，并打印 `stopped_by_threshold` 位姿后退出；
- 收到 `0` 或其他值：仅日志记录，不停止；
- 即使暂时没有消息，也会周期性输出监听状态日志。

### 11.1 手动发送停止信号

另开终端执行：

```bash
source /home/hanmo/code/catkin_ws/devel/setup.bash
rostopic pub /threshold_detect std_msgs/Int32 "data: 1" -1
```

---

## 12. 已知问题与说明

1. 某些版本的 `jaka_driver` 不包含 `jaka_state_adapter_node` 可执行文件。  
   因此当前 launch 已避免强依赖该节点，防止因找不到可执行文件导致整套启动失败。

2. 若 `openloop_move_jaka4` 一直等待 `/jaka4/jaka_driver/tool_position`，需要继续确认：  
   - 当前 `jaka_driver` 实际发布的话题名；  
   - 是否与本项目默认话题一致（默认 `/jaka4/jaka_driver/tool_position` 与 service `/jaka4/jaka_driver/linear_move`）。

---

## 13. 关键 launch 参数（当前版本）

- `enable_jaka1~enable_jaka4`：分别控制 4 台机械臂是否启动
- `jaka1_ip~jaka4_ip`：各机械臂 IP
- `start_jaka4_demo`：是否同时启动 `openloop_move_jaka4`
- `urdf_file`：URDF 文件路径（默认值如下）
- `line_distance_mm`：连续直线运动距离（默认 80mm）
- `direction_x/y/z`：方向向量（自动归一化）
- `direction_frame`：`base` 或 `tool`
- `line_speed_mm_s`：直线速度
- `line_acc_mm_s2`：直线加速度
- `threshold_topic`：阈值监听话题（默认 `/threshold_detect`）
- `stop_move_service`：停止服务（默认 `/jaka4/jaka_driver/stop_move`）

```xml
<arg name="urdf_file" default="$(find threshold_listener_jaka)/urdf/jaka_zu3.urdf"/>
```

## 14. 驱动 IP 参数名对齐说明

`jaka_driver` 实际读取的参数名是 `ip`（不是 `robot_ip`）。

因此本项目 `multi_jaka_openloop.launch` 已按 `ip` 传参，例如：

```xml
<param name="ip" value="$(arg jaka4_ip)"/>
```



## 15. 连接方式对齐说明

`multi_jaka_openloop.launch` 已按成功项目的连接方式对齐：

- group 级参数：`ip` + `robot_description`；
- driver 节点内同时传 `ip` 与 `robot_ip`；
- driver 节点内加入常用控制/状态话题 remap；
- 状态适配节点已移除（当前项目不再依赖外部适配节点包）；
- `robot_state_publisher` 使用 `publish_frequency=100.0` 并 remap `/joint_states -> joint_states`。

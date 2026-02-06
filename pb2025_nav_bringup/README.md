# pb2025_nav_bringup 启动文件详解

## 目录
1. [概述](#概述)
2. [启动文件列表](#启动文件列表)
3. [启动参数详解](#启动参数详解)
4. [启动文件详解](#启动文件详解)
   - [4.1 bringup_launch.py - 主启动文件](#41-bringup_launchpy---主启动文件)
   - [4.2 joy_teleop_launch.py - 手柄控制启动](#42-joy_teleop_launchpy---手柄控制启动)
   - [4.3 localization_launch.py - 定位启动](#43-localization_launchpy---定位启动)
   - [4.4 navigation_launch.py - 导航启动](#44-navigation_launchpy---导航启动)
   - [4.5 rm_navigation_simulation_launch.py - 仿真启动](#45-rm_navigation_simulation_launchpy---仿真启动)
   - [4.6 rm_navigation_reality_launch.py - 实车启动](#46-rm_navigation_reality_launchpy---实车启动)
   - [4.7 rm_multi_navigation_simulation_launch.py - 多机器人仿真启动](#47-rm_multi_navigation_simulation_launchpy---多机器人仿真启动)
   - [4.8 robot_state_publisher_launch.py - 机器人状态发布器启动](#48-robot_state_publisher_launchpy---机器人状态发布器启动)
   - [4.9 rviz_launch.py - RViz 可视化启动](#49-rviz_launchpy---rviz-可视化启动)
   - [4.10 slam_launch.py - SLAM 建图启动](#410-slam_launchpy---slam-建图启动)
5. [启动实例](#启动实例)
6. [常见问题](#常见问题)

---

## 概述

`pb2025_nav_bringup` 包含了哨兵机器人导航系统的所有启动文件，用于启动仿真和实车环境下的导航功能。这些启动文件基于 ROS2 Launch 系统，支持灵活的参数配置和模块化启动。

**主要功能：**
- 支持仿真和实车两种模式
- 支持 SLAM 建图和导航两种模式
- 支持单机器人和多机器人场景
- 支持命名空间（Namespace）隔离
- 支持 Composable Node 和普通 Node 两种启动方式

---

## 启动文件列表

| 启动文件 | 功能描述 | 适用场景 |
|----------|----------|----------|
| [bringup_launch.py](#41-bringup_launchpy---主启动文件) | 主启动文件，集成所有导航模块 | 通用 |
| [joy_teleop_launch.py](#42-joy_teleop_launchpy---手柄控制启动) | 启动手柄控制节点 | 通用 |
| [localization_launch.py](#43-localization_launchpy---定位启动) | 启动定位模块（small_gicp） | 实车导航 |
| [navigation_launch.py](#44-navigation_launchpy---导航启动) | 启动导航核心模块 | 通用 |
| [rm_navigation_simulation_launch.py](#45-rm_navigation_simulation_launchpy---仿真启动) | 仿真环境完整启动 | 仿真 |
| [rm_navigation_reality_launch.py](#46-rm_navigation_reality_launchpy---实车启动) | 实车环境完整启动 | 实车 |
| [rm_multi_navigation_simulation_launch.py](#47-rm_multi_navigation_simulation_launchpy---多机器人仿真启动) | 多机器人仿真启动 | 多机器人仿真 |
| [robot_state_publisher_launch.py](#48-robot_state_publisher_launchpy---机器人状态发布器启动) | 启动机器人状态发布器 | 实车 |
| [rviz_launch.py](#49-rviz_launchpy---rviz-可视化启动) | 启动 RViz 可视化工具 | 通用 |
| [slam_launch.py](#410-slam_launchpy---slam-建图启动) | 启动 SLAM 建图模块 | 建图 |

---

## 启动参数详解

### 通用参数

以下参数在多个启动文件中通用：

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|---------|--------|
| `namespace` | string | "red_standard_robot1" | 顶级命名空间，用于多机器人隔离 |
| `use_sim_time` | bool | 仿真: True, 实车: False | 是否使用仿真时钟（Gazebo） |
| `params_file` | string | 自动填充 | ROS2 参数文件路径 |
| `autostart` | bool | True | 是否自动启动导航栈 |
| `use_composition` | bool | True | 是否使用 Composable Node 形式启动 |
| `use_respawn` | bool | False | 节点崩溃时是否自动重启（仅在非 Composition 模式下有效） |
| `log_level` | string | "info" | 日志级别（debug, info, warn, error） |
| `rviz_config` | string | 自动填充 | RViz 配置文件路径 |

### 仿真专用参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|---------|--------|
| `world` | string | "rmuc_2025" | 仿真世界名称（rmul_2024, rmuc_2024, rmul_2025, rmuc_2025） |
| `map` | string | 自动填充 | 地图文件路径（基于 world 参数构建） |
| `prior_pcd_file` | string | 自动填充 | 先验点云文件路径（基于 world 参数构建） |

### 实车专用参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|---------|--------|
| `use_robot_state_pub` | bool | False | 是否启动机器人状态发布器 |

### 手柄控制专用参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|---------|--------|
| `joy_vel` | string | "cmd_vel" | 发布速度指令的话题名称 |
| `joy_dev` | int | 0 | 手柄设备 ID |
| `joy_config_file` | string | 自动填充 | 手柄配置文件路径 |

---

## 启动文件详解

### 4.1 bringup_launch.py - 主启动文件

#### 功能概述

`bringup_launch.py` 是主启动文件，负责集成和启动所有导航相关的模块。它根据参数条件启动 SLAM 或定位模块，并启动导航核心功能。

#### 启动的模块

**SLAM 模式（slam=True）：**
- [slam_launch.py](#410-slam_launchpy---slam-建图启动) - SLAM 建图模块
- [navigation_launch.py](#44-navigation_launchpy---导航启动) - 导航核心模块

**导航模式（slam=False）：**
- [localization_launch.py](#43-localization_launchpy---定位启动) - 定位模块（small_gicp）
- [navigation_launch.py](#44-navigation_launchpy---导航启动) - 导航核心模块

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `slam` | bool | "False" | 是否运行 SLAM |
| `map` | string | - | 要加载的地图文件路径 |
| `prior_pcd_file` | string | - | 要加载的先验点云文件路径 |
| `use_sim_time` | bool | "false" | 是否使用仿真时钟 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `autostart` | bool | "true" | 是否自动启动导航栈 |
| `use_composition` | bool | "True" | 是否使用组合节点启动 |
| `use_respawn` | bool | "False" | 节点崩溃时是否自动重启 |
| `log_level` | string | "info" | 日志级别 |

#### 启动示例

**示例 1：启动 SLAM 模式**

```bash
ros2 launch pb2025_nav_bringup bringup_launch.py \
    namespace:=red_standard_robot1 \
    slam:=True \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml
```

**示例 2：启动导航模式**

```bash
ros2 launch pb2025_nav_bringup bringup_launch.py \
    namespace:=red_standard_robot1 \
    slam:=False \
    map:=/path/to/map.yaml \
    prior_pcd_file:=/path/to/prior.pcd \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml
```

---

### 4.2 joy_teleop_launch.py - 手柄控制启动

#### 功能概述

`joy_teleop_launch.py` 启动手柄控制节点，允许通过游戏手柄控制机器人移动。

#### 启动的模块

- `joy_node` - 手柄驱动节点（来自 joy 包）
- `pb_teleop_twist_joy_node` - 手柄速度转换节点

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `use_sim_time` | bool | "false" | 是否使用仿真时钟 |
| `joy_vel` | string | "cmd_vel" | 发布速度指令的话题名称 |
| `joy_config_file` | string | nav2_params.yaml | 手柄配置文件路径 |
| `joy_dev` | int | 0 | 手柄设备 ID |

#### 启动示例

```bash
ros2 launch pb2025_nav_bringup joy_teleop_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    joy_vel:=cmd_vel \
    joy_dev:=0
```

#### 手柄按键映射

默认使用 PS4 手柄，按键映射可在 `nav2_params.yaml` 的 `teleop_twist_joy_node` 部分配置：

| 按键 | 功能 |
|--------|------|
| 左摇杆 X 轴 | 前后移动 |
| 左摇杆 Y 轴 | 左右移动 |
| 右摇杆 X 轴 | 旋转 |
| R1/R2 | 加速/减速 |
| L1/L2 | 切换模式 |

---

### 4.3 localization_launch.py - 定位启动

#### 功能概述

`localization_launch.py` 启动定位模块，使用 small_gicp 算法进行重定位。

#### 启动的模块

- `map_saver_server` - 地图保存服务器
- `lifecycle_manager_slam` - 生命周期管理器
- `pointcloud_to_laserscan_node` - 点云转激光扫描节点
- `sync_slam_toolbox_node` - SLAM 工具箱同步节点
- `point_lio` - 点云里程计节点
- `static_transform_publisher` - 静态变换发布器（map → odom）

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `use_sim_time` | bool | "True" | 是否使用仿真时钟 |
| `autostart` | bool | "True" | 是否自动启动导航栈 |
| `use_respawn` | bool | "False" | 节点崩溃时是否自动重启 |
| `log_level` | string | "info" | 日志级别 |

#### 启动示例

```bash
ros2 launch pb2025_nav_bringup localization_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml
```

#### 功能说明

- **map_saver_server**：提供地图保存服务
- **pointcloud_to_laserscan_node**：将地形地图转换为激光扫描数据，用于障碍物表示
- **sync_slam_toolbox_node**：同步 SLAM 工具箱数据
- **point_lio**：运行点云里程计算法，估计机器人位姿
- **static_transform_publisher**：发布 map 到 odom 的静态变换

---

### 4.4 navigation_launch.py - 导航启动

#### 功能概述

`navigation_launch.py` 启动导航核心模块，包括控制器、规划器、行为树等 NAV2 核心组件。

#### 启动的模块

**普通节点模式（use_composition=False）：**
- `loam_interface_node` - 激光里程计接口节点
- `sensor_scan_generation_node` - 传感器扫描生成节点
- `fake_vel_transform_node` - 速度变换节点
- `controller_server` - 控制器服务器
- `smoother_server` - 平滑器服务器
- `planner_server` - 规划器服务器
- `behavior_server` - 行为服务器
- `bt_navigator` - 行为树导航器
- `waypoint_follower` - 航点跟随器
- `velocity_smoother` - 速度平滑器
- `lifecycle_manager_navigation` - 导航生命周期管理器

**组合节点模式（use_composition=True）：**
- 所有上述节点作为 Composable Node 加载到 `nav2_container` 中

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `use_sim_time` | bool | "false" | 是否使用仿真时钟 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `autostart` | bool | "true" | 是否自动启动导航栈 |
| `use_composition` | bool | "False" | 是否使用组合节点启动 |
| `container_name` | string | "nav2_container" | 组合节点容器名称 |
| `use_respawn` | bool | "False" | 节点崩溃时是否自动重启 |
| `log_level` | string | "info" | 日志级别 |

#### 启动示例

**示例 1：普通节点模式**

```bash
ros2 launch pb2025_nav_bringup navigation_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml \
    use_composition:=False
```

**示例 2：组合节点模式**

```bash
ros2 launch pb2025_nav_bringup navigation_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml \
    use_composition:=True \
    container_name:=nav2_container
```

#### 功能说明

- **loam_interface_node**：处理激光里程计数据，进行坐标转换
- **sensor_scan_generation_node**：生成传感器扫描数据，发布 TF 和里程计
- **fake_vel_transform_node**：处理云台扫描模式下的速度变换
- **controller_server**：执行路径跟踪控制
- **smoother_server**：平滑路径和速度
- **planner_server**：生成全局路径规划
- **behavior_server**：执行导航行为（如恢复、旋转等）
- **bt_navigator**：管理导航行为树
- **waypoint_follower**：跟随航点路径
- **velocity_smoother**：平滑速度输出

#### 话题重映射

- `cmd_vel` → `cmd_vel_controller`：控制器输入
- `cmd_vel_smoothed` → `cmd_vel_nav2_result`：导航器输出

---

### 4.5 rm_navigation_simulation_launch.py - 仿真启动

#### 功能概述

`rm_navigation_simulation_launch.py` 是仿真环境的完整启动文件，集成了所有必要的模块。

#### 启动的模块

- `ign_sim_pointcloud_tool_node` - 仿真点云转换工具
- [bringup_launch.py](#41-bringup_launchpy---主启动文件) - 主启动文件
- [joy_teleop_launch.py](#42-joy_teleop_launchpy---手柄控制启动) - 手柄控制
- [rviz_launch.py](#49-rviz_launchpy---rviz-可视化启动) - RViz 可视化

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "red_standard_robot1" | 顶级命名空间 |
| `slam` | bool | "False" | 是否运行 SLAM |
| `world` | string | "rmuc_2025" | 仿真世界名称 |
| `map` | string | 自动填充 | 地图文件路径 |
| `prior_pcd_file` | string | 自动填充 | 先验点云文件路径 |
| `use_sim_time` | bool | "True" | 是否使用仿真时钟 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `autostart` | bool | "true" | 是否自动启动导航栈 |
| `use_composition` | bool | "True" | 是否使用组合节点启动 |
| `use_respawn` | bool | "False" | 节点崩溃时是否自动重启 |
| `rviz_config` | string | nav2_default_view.rviz | RViz 配置文件路径 |
| `use_rviz` | bool | "True" | 是否启动 RViz |

#### 启动示例

**示例 1：导航模式**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    namespace:=red_standard_robot1 \
    world:=rmuc_2025 \
    slam:=False
```

**示例 2：SLAM 模式**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    namespace:=red_standard_robot1 \
    world:=rmuc_2025 \
    slam:=True
```

**示例 3：不启动 RViz**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    namespace:=red_standard_robot1 \
    world:=rmuc_2025 \
    slam:=False \
    use_rviz:=False
```

#### 功能说明

- **ign_sim_pointcloud_tool_node**：将仿真器输出的点云转换为 point_lio 所需格式
- **bringup_launch.py**：启动主导航模块
- **joy_teleop_launch.py**：启动手柄控制
- **rviz_launch.py**：启动 RViz 可视化

---

### 4.6 rm_navigation_reality_launch.py - 实车启动

#### 功能概述

`rm_navigation_reality_launch.py` 是实车环境的完整启动文件，集成了所有必要的模块。

#### 启动的模块

- `livox_ros_driver2_node` - Livox 激光雷达驱动
- [robot_state_publisher_launch.py](#48-robot_state_publisher_launchpy---机器人状态发布器启动) - 机器人状态发布器（可选）
- [bringup_launch.py](#41-bringup_launchpy---主启动文件) - 主启动文件
- [joy_teleop_launch.py](#42-joy_teleop_launchpy---手柄控制启动) - 手柄控制
- [rviz_launch.py](#49-rviz_launchpy---rviz-可视化启动) - RViz 可视化

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `slam` | bool | "False" | 是否运行 SLAM |
| `world` | string | "rmul_2024" | 世界名称（用于加载地图和点云） |
| `map` | string | 自动填充 | 地图文件路径 |
| `prior_pcd_file` | string | 自动填充 | 先验点云文件路径 |
| `use_sim_time` | bool | "False" | 是否使用仿真时钟 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `autostart` | bool | "true" | 是否自动启动导航栈 |
| `use_composition` | bool | "True" | 是否使用组合节点启动 |
| `use_respawn` | bool | "False" | 节点崩溃时是否自动重启 |
| `rviz_config` | string | nav2_default_view.rviz | RViz 配置文件路径 |
| `use_rviz` | bool | "True" | 是否启动 RViz |
| `use_robot_state_pub` | bool | "False" | 是否启动机器人状态发布器 |

#### 启动示例

**示例 1：导航模式**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    namespace:=red_standard_robot1 \
    world:=my_map \
    slam:=False \
    use_robot_state_pub:=False
```

**示例 2：SLAM 模式**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    namespace:=red_standard_robot1 \
    world:=my_map \
    slam:=True \
    use_robot_state_pub:=False
```

**示例 3：启动机器人状态发布器**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    namespace:=red_standard_robot1 \
    world:=my_map \
    slam:=False \
    use_robot_state_pub:=True
```

#### 功能说明

- **livox_ros_driver2_node**：驱动 Livox Mid360 激光雷达
- **robot_state_publisher_launch.py**：发布机器人关节状态（如云台角度）
- **bringup_launch.py**：启动主导航模块
- **joy_teleop_launch.py**：启动手柄控制
- **rviz_launch.py**：启动 RViz 可视化

---

### 4.7 rm_multi_navigation_simulation_launch.py - 多机器人仿真启动

#### 功能概述

`rm_multi_navigation_simulation_launch.py` 启动多个机器人的仿真环境，每个机器人使用独立的命名空间。

#### 启动的模块

为每个机器人启动：
- [rm_navigation_simulation_launch.py](#45-rm_navigation_simulation_launchpy---仿真启动) - 单机器人仿真启动

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `world` | string | "rmul_2024" | 仿真世界名称 |
| `map` | string | 自动填充 | 地图文件路径 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `autostart` | bool | "True" | 是否自动启动导航栈 |
| `rviz_config` | string | nav2_default_view.rviz | RViz 配置文件路径 |
| `use_rviz` | bool | "True" | 是否启动 RViz |
| `log_settings` | bool | "true" | 是否显示日志设置 |
| `robots` | string | - | 机器人配置列表 |

#### robots 参数格式

`robots` 参数使用以下格式：

```bash
robots:=" \
    robot_name1={x: 0.0, y: 0.0, yaw: 0.0}; \
    robot_name2={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
```

每个机器人的配置参数：

| 参数 | 类型 | 描述 |
|------|------|--------|
| `x` | float | 初始 X 坐标 |
| `y` | float | 初始 Y 坐标 |
| `z` | float | 初始 Z 坐标（可选） |
| `roll` | float | 初始横滚角（可选） |
| `pitch` | float | 初始俯仰角（可选） |
| `yaw` | float | 初始偏航角 |

#### 启动示例

**示例 1：两个机器人**

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
        red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
        blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
```

**示例 2：四个机器人（带完整位姿）**

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
        robot1={x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}; \
        robot2={x: 1.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 1.5707}; \
        robot3={x: 1.0, y: 1.0, z: 0.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}; \
        robot4={x: 1.0, y: 1.0, z: 0.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}; \
    "
```

**示例 3：不启动 RViz**

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
        red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
        blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
    " \
    use_rviz:=False
```

#### 功能说明

- 为每个机器人创建独立的命名空间
- 每个机器人有独立的导航栈
- 支持任意数量的机器人
- 每个机器人可以配置不同的初始位姿

---

### 4.8 robot_state_publisher_launch.py - 机器人状态发布器启动

#### 功能概述

`robot_state_publisher_launch.py` 启动机器人状态发布器，用于发布机器人关节状态（如云台角度）。

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `use_sim_time` | bool | "false" | 是否使用仿真时钟 |

#### 启动示例

```bash
ros2 launch pb2025_nav_bringup robot_state_publisher_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False
```

#### 功能说明

- 发布机器人关节状态（如 gimbal_yaw, gimbal_pitch）
- 用于维护 TF 树
- 通常与串口通信模块配合使用

---

### 4.9 rviz_launch.py - RViz 可视化启动

#### 功能概述

`rviz_launch.py` 启动 RViz 可视化工具，用于实时查看机器人状态和导航信息。

#### 启动的模块

- `rviz2` - RViz 可视化节点

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `rviz_config` | string | nav2_default_view.rviz | RViz 配置文件路径 |

#### 启动示例

```bash
ros2 launch pb2025_nav_bringup rviz_launch.py \
    namespace:=red_standard_robot1 \
    rviz_config:=/path/to/nav2_default_view.rviz
```

#### 功能说明

- 实时显示机器人模型
- 显示点云数据
- 显示地图
- 显示路径规划
- 显示 TF 树

---

### 4.10 slam_launch.py - SLAM 建图启动

#### 功能概述

`slam_launch.py` 启动 SLAM 建图模块，用于构建环境地图。

#### 启动的模块

- `map_saver_server` - 地图保存服务器
- `lifecycle_manager_slam` - 生命周期管理器
- `pointcloud_to_laserscan_node` - 点云转激光扫描节点
- `sync_slam_toolbox_node` - SLAM 工具箱同步节点
- `point_lio` - 点云里程计节点（建图模式）
- `static_transform_publisher` - 静态变换发布器（map → odom）

#### 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|---------|--------|
| `namespace` | string | "" | 顶级命名空间 |
| `params_file` | string | nav2_params.yaml | ROS2 参数文件路径 |
| `use_sim_time` | bool | "True" | 是否使用仿真时钟 |
| `autostart` | bool | "True" | 是否自动启动导航栈 |
| `use_respawn` | bool | "False" | 节点崩溃时是否自动重启 |
| `log_level` | string | "info" | 日志级别 |

#### 启动示例

```bash
ros2 launch pb2025_nav_bringup slam_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml
```

#### 功能说明

- **point_lio**：在建图模式下运行，构建点云地图
- **map_saver_server**：提供地图保存服务
- **pointcloud_to_laserscan_node**：将点云转换为激光扫描，用于障碍物检测
- **sync_slam_toolbox_node**：同步 SLAM 工具箱数据
- **static_transform_publisher**：发布 map 到 odom 的静态变换

#### 保存地图

建图完成后，使用以下命令保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -r __ns:=/red_standard_robot1
```

保存的文件：
- `my_map.yaml` - 地图元数据
- `my_map.pgm` - 地图图像
- `my_map.pcd` - 点云地图（自动保存到 point_lio/PCD/ 目录）

---

## 启动实例

### 实例 1：单机器人仿真导航

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmuc_2025 \
    slam:=False
```

**说明：**
- 启动仿真环境
- 加载 rmuc_2025 地图
- 启动导航模式
- 启动 RViz 可视化
- 启动手柄控制

### 实例 2：单机器人仿真 SLAM

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmuc_2025 \
    slam:=True
```

**说明：**
- 启动仿真环境
- 启动 SLAM 建图模式
- 启动 RViz 可视化
- 启动手柄控制

### 实例 3：单机器人实车导航

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    world:=my_map \
    slam:=False \
    use_robot_state_pub:=False
```

**说明：**
- 启动实车环境
- 加载 my_map 地图
- 启动导航模式
- 启动 Livox 激光雷达驱动
- 启动 RViz 可视化
- 启动手柄控制

### 实例 4：单机器人实车 SLAM

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    world:=my_map \
    slam:=True \
    use_robot_state_pub:=False
```

**说明：**
- 启动实车环境
- 启动 SLAM 建图模式
- 启动 Livox 激光雷达驱动
- 启动 RViz 可视化
- 启动手柄控制

### 实例 5：多机器人仿真

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
        red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
        blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
```

**说明：**
- 启动两个机器人的仿真环境
- 每个机器人有独立的命名空间
- 每个机器人有独立的导航栈
- 启动 RViz 可视化（每个机器人一个窗口）

### 实例 6：仅启动导航模块

```bash
ros2 launch pb2025_nav_bringup navigation_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    params_file:=/path/to/nav2_params.yaml \
    use_composition:=False
```

**说明：**
- 仅启动导航核心模块
- 不启动传感器驱动
- 不启动可视化
- 适用于调试和测试

### 实例 7：仅启动手柄控制

```bash
ros2 launch pb2025_nav_bringup joy_teleop_launch.py \
    namespace:=red_standard_robot1 \
    use_sim_time:=False \
    joy_vel:=cmd_vel
```

**说明：**
- 仅启动手柄控制
- 不启动导航模块
- 适用于手动控制测试

### 实例 8：仅启动 RViz

```bash
ros2 launch pb2025_nav_bringup rviz_launch.py \
    namespace:=red_standard_robot1 \
    rviz_config:=/path/to/nav2_default_view.rviz
```

**说明：**
- 仅启动 RViz 可视化
- 不启动其他模块
- 适用于远程可视化

---

## 常见问题

### Q1: 如何查看 TF 树？

**A:** 使用以下命令查看特定命名空间的 TF 树：

```bash
ros2 run rqt_tf_tree rqt_tf_tree \
    --ros-args \
    -r /tf:=tf \
    -r /tf_static:=tf_static \
    -r __ns:=/red_standard_robot1
```

### Q2: 如何保存地图？

**A:** 建图模式下，使用以下命令保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -r __ns:=/red_standard_robot1
```

### Q3: 如何切换到导航模式？

**A:** 停止当前 SLAM 进程，然后使用导航模式启动：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    world:=my_map \
    slam:=False
```

### Q4: 如何修改参数？

**A:** 有两种方式修改参数：

**方式 1：通过命令行参数**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    namespace:=my_namespace \
    world:=my_world
```

**方式 2：修改参数文件**

编辑 `nav2_params.yaml` 文件，然后重新启动。

### Q5: 如何禁用 RViz？

**A:** 使用 `use_rviz:=False` 参数：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmuc_2025 \
    slam:=False \
    use_rviz:=False
```

### Q6: 如何使用 Composable Node？

**A:** 设置 `use_composition:=True` 参数：

```bash
ros2 launch pb2025_nav_bringup navigation_launch.py \
    namespace:=red_standard_robot1 \
    use_composition:=True
```

Composable Node 的优势：
- 更低的资源消耗
- 更快的启动速度
- 更好的进程间通信

### Q7: 多机器人如何避免冲突？

**A:** 使用不同的命名空间：

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    robots:=" \
        robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
        robot2={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
```

每个机器人使用独立的命名空间（robot1, robot2），避免话题和节点名称冲突。

### Q8: 如何调试启动问题？

**A:** 使用以下方法：

**方法 1：增加日志级别**

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmuc_2025 \
    slam:=False \
    log_level:=debug
```

**方法 2：逐个启动模块**

```bash
# 终端 1：启动 RViz
ros2 launch pb2025_nav_bringup rviz_launch.py

# 终端 2：启动导航模块
ros2 launch pb2025_nav_bringup navigation_launch.py
```

**方法 3：检查话题和节点**

```bash
ros2 topic list
ros2 node list
```

### Q9: 如何设置初始位姿？

**A:** 在多机器人启动时，通过 `robots` 参数设置：

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    robots:=" \
        my_robot={x: 1.0, y: 2.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 1.57}; \
    "
```

### Q10: 如何使用自定义参数文件？

**A:** 通过 `params_file` 参数指定：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmuc_2025 \
    slam:=False \
    params_file:=/path/to/my_params.yaml
```

---

## 附录

### A. 参数文件位置

默认参数文件位置：

| 环境 | 路径 |
|------|------|
| 仿真 | `pb2025_nav_bringup/config/simulation/nav2_params.yaml` |
| 实车 | `pb2025_nav_bringup/config/reality/nav2_params.yaml` |

### B. 地图文件位置

默认地图文件位置：

| 环境 | 路径 |
|------|------|
| 仿真 | `pb2025_nav_bringup/map/simulation/` |
| 实车 | `pb2025_nav_bringup/map/reality/` |

### C. RViz 配置文件位置

默认 RViz 配置文件位置：

| 环境 | 路径 |
|------|------|
| 通用 | `pb2025_nav_bringup/rviz/nav2_default_view.rviz` |

### D. 命名空间使用

所有启动文件都支持命名空间参数，用于多机器人隔离：

```bash
# 单机器人
namespace:=red_standard_robot1

# 多机器人
namespace:=robot1
namespace:=robot2
```

### E. 话题重映射

TF 话题需要特殊重映射：

```python
remappings = [
    ('/tf', 'tf'),
    ('/tf_static', 'tf_static')
]
```

---

## 更新日志

### 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|---------|
| 1.0.0 | 2025-01-01 | 初始版本 |

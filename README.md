# RViz2 Joint State Monitor Panel

一个基于 **ROS 2 Humble、C++17 和 Qt5 Widgets** 的通用关节状态监视面板，可以直接停靠在 RViz2 中，与 RobotModel、TF 和 MoveIt MotionPlanning 一起使用。

面板只订阅 `sensor_msgs/msg/JointState`，不发送运动控制指令。它复用 RViz 的 ROS 节点和 Qt 事件循环，无需独立监视进程。

## 功能

- 默认订阅 `/joint_states`，支持在界面中修改话题和订阅 QoS。
- 根据关节名称自动识别数据，支持六轴、七轴、双臂和混合关节数量。
- 按首次出现顺序稳定排列；消息乱序不会导致数值错配或列表跳动。
- 支持指定名称过滤及固定显示顺序，未收到的位置显示 `--`。
- 旋转关节支持 `deg / rad` 切换，直线关节显示 `m`，未知类型显示 `raw`。
- 显示有效数据接收频率、关节数量、发布者数量及最近更新时间。
- 支持全局超时、独立关节过期和非法数据诊断；停更保留最后有效值。
- 可从 YAML 加载关节类型、真实限位和不同机器人配置。
- 设置随 `.rviz` 配置保存，下次启动 RViz 自动恢复。

插件标识为 **`joint_state_rviz_panel/JointStatePanel`**，基类为 `rviz_common::Panel`。

## 环境与依赖

已验证环境：Ubuntu 22.04、ROS 2 Humble、Qt 5.15.3、GCC 11、yaml-cpp 0.7。

先完成 ROS 2 Humble 安装，再安装依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions \
  qtbase5-dev libyaml-cpp-dev ros-humble-rviz2 \
  ros-humble-ament-cmake-gtest
```

不依赖 MoveIt 或特定机械臂驱动；接入 MoveIt 时使用已有的 RViz 和 JointState 话题即可。

## 下载与编译

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/nan097/RViz2-Joint-State-Monitor-Panel.git
cd ~/ros2_ws
colcon build --packages-select joint_state_rviz_panel --symlink-install
source install/setup.bash
```

仓库根目录就是 ROS 软件包，包名为 `joint_state_rviz_panel`，不需要再复制一层同名目录。

只构建插件、不构建测试宿主时，可增加 `--cmake-args -DBUILD_TESTING=OFF`。

## 加载面板

在已加载工作空间环境的终端启动 RViz：

```bash
rviz2
```

然后：

1. 选择 **Panels → Add New Panel**。
2. 展开 **joint_state_rviz_panel**，选择 **JointStatePanel**。
3. 将面板拖到右侧，或按需要浮动、停靠。
4. 输入话题并点击“应用”，设置单位、过滤条件和 QoS。
5. 使用 **File → Save Config As** 保存新的 `.rviz` 文件。

也可以直接打开示例配置：

```bash
rviz2 -d "$(ros2 pkg prefix joint_state_rviz_panel)/share/joint_state_rviz_panel/rviz/joint_monitor.rviz"
```

示例包含 Grid 和右侧关节监视面板，固定坐标系为 `world`。没有对应 TF 时，三维显示可能提示坐标系异常，不影响关节状态订阅。

## 接入已有 MoveIt 工程

在**启动原工程 launch 的同一个终端**中，依次加载 ROS、机器人工作空间及本插件工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
source ~/ros2_ws/install/local_setup.bash
# 接着运行你原来的 MoveIt 启动命令。
```

在原有 RViz 中添加面板并保存配置，然后将原 launch 中 RViz 的 `-d` 参数或已有的 `rviz_config` 参数指向该 `.rviz` 文件。后续仍只启动原工程，无需另开监视进程。

**在另一个终端执行 source 不会改变已运行 RViz 的环境。** 安装或更新插件后，应从正确加载环境的终端重新启动 RViz。不要用本仓库的通用示例覆盖现有 MoveIt 显示配置。

## 界面使用

| 设置 | 作用 |
|---|---|
| 话题输入框与“应用” | 修改订阅话题；成功切换后清除旧数据 |
| 旋转单位 | 在 `deg` 与 `rad` 之间切换，仅影响旋转关节 |
| QoS | 默认 `best_effort`，也可选择 `reliable` |
| 自动全部 | 展示全部有效关节，保持首次出现顺序 |
| 指定过滤 | 输入逗号分隔的名称，按此顺序显示；编辑后点击“应用” |
| 加载 YAML | 加载机器人配置，在下拉框中选择相应 profile |
| 未知类型按旋转关节显示 | 明确启用类型推断，界面同时标注推断状态 |

切换话题、模式、profile 或 QoS 会清空旧缓存。话题非法或配置校验失败时保留原生效配置，并在面板中显示错误。

### 类型与单位

`JointState` 本身不包含关节类型或限位，程序不会默认将所有位置解释为角度。

| 类型 | 显示单位 | 位置条 |
|---|---|---|
| `revolute` | `deg` 或 `rad` | 配置有效上下限时显示 |
| `continuous` | `deg` 或 `rad` | 不显示物理限位条 |
| `prismatic` | `m` | 配置有效上下限时显示 |
| `unknown` / 未配置 | `raw` | 不虚构限位 |

旋转限位按弧度配置，直线限位按米配置；上下限必须同时提供、数值有限且下限小于上限。超限只提示，不执行控制。

## YAML 配置

完整示例见 `config/monitor.yaml`。在界面点击“加载 YAML”选择文件即可。

```yaml
monitor:
  topic: /joint_states
  refresh_hz: 20
  qos_reliability: best_effort
  warn_after_sec: 1.0
  timeout_after_sec: 2.0
  default_rotation_unit: deg
  mode: configured
  robot_profile: my_robot
  assume_unknown_revolute: false

robots:
  my_robot:
    display_name: My Robot
    joint_names: [shoulder_joint, slide_joint]
    joints:
      # 名称与限位只是示例，使用前请按实际驱动和 URDF 修改。
      shoulder_joint: {type: revolute, lower: -3.14, upper: 3.14}
      slide_joint: {type: prismatic, lower: 0.0, upper: 0.5}
```

`mode` 可取 `auto` 或 `configured`；刷新率允许 1–100 Hz。超时阈值要求 `0 < warn_after_sec < timeout_after_sec`。

`.rviz` 会保存话题、单位、过滤名称、QoS、类型推断选项和当前 profile，并内嵌完整机器人定义、限位和阈值。因此保存后无需继续依赖最初加载的 YAML 文件路径。

## 数据处理规则

- 使用 `name[i]` 关联 `position[i]`，不按固定轴数或固定下标解释数据。
- `position` 为空或长度与 `name` 不一致时忽略该条位置更新。
- 空名称和 NaN/Inf 逐项忽略；重复名称采用首次出现项，并记录诊断。
- 部分更新不会将缺席关节清零，各关节保留自己的有效值和接收时间。
- 超时依据本机单调时钟，不依赖可能暂停的 `/clock` 或消息时间戳。
- 默认 1 秒无有效更新提示迟缓，2 秒提示超时；恢复发布后自动恢复。
- Hz 基于最近 3 秒有效消息间隔计算，至少两个样本且跨度达到 0.2 秒才显示估计值。
- 全局频率统计整个话题；过滤列表外的有效数据也计入频率，各显示关节独立判断过期。
- 发布者数量只用于诊断，不代表数据有效；多个发布者共享话题时提示用命名空间隔离。

ROS 回调只写入互斥缓存，Qt 定时器读取独立快照后更新界面。切换订阅时更换独立缓存，旧回调不会污染新话题；销毁面板时停止定时器并释放订阅，不改变 RViz 的全局 ROS 生命周期。

## 模拟数据

另开一个终端并加载相同工作空间环境后，选择一个测试场景：

```bash
# 模拟七个关节，随机调整消息内名称顺序。
ros2 run joint_state_rviz_panel mock_joint_states_publisher.py --count 7 --shuffle

# 模拟十四个关节，三秒后仅更新其中一部分。
ros2 run joint_state_rviz_panel mock_joint_states_publisher.py --count 14 --hz 100 --partial

# 四秒后暂停三秒，再自动恢复。
ros2 run joint_state_rviz_panel mock_joint_states_publisher.py --pause-after 4 --pause-for 3

# 模拟非法数据。
ros2 run joint_state_rviz_panel mock_joint_states_publisher.py --invalid nan

# 左右旋转关节与直线关节混合，对应 example_mixed 配置。
ros2 run joint_state_rviz_panel mock_joint_states_publisher.py --mixed
```

`--invalid` 支持 `empty / mismatch / nan / inf / duplicate`；`--topic` 可指定其他测试话题。切换场景前停止上一发布器。

模拟发布器只用于测试。连接真实机器人时直接使用驱动已发布的 JointState，无需另外启动它。

## 测试

在工作空间根目录执行：

```bash
colcon test --packages-select joint_state_rviz_panel --event-handlers console_direct+
colcon test-result --verbose
```

已验证 **2 个 CTest 目标、11 个 GTest 用例**，覆盖名称映射、部分更新、非法数据、频率估计、插件发现、配置往返恢复、6/7/14 关节、单位换算、超时恢复、切话题与反复销毁面板。

真实 RViz 宿主测试还验证了右侧停靠、列表不重叠、保存重载后继续更新以及 Grid 保留。需要可用桌面和 OpenGL，在工作空间根目录运行：

```bash
QT_QPA_PLATFORM=xcb ROS_LOG_DIR=/tmp/joint-panel-host-logs \
  build/joint_state_rviz_panel/rviz_host_smoke \
  src/RViz2-Joint-State-Monitor-Panel/rviz/joint_monitor.rviz

python3 src/RViz2-Joint-State-Monitor-Panel/test/smoke_rviz.py
```

测试宿主仅在 `BUILD_TESTING=ON` 时构建，不安装为产品入口。默认 Qt 测试使用 offscreen，不要求显示服务器。尚未完成各型号实机、多机 DDS 和长时间内存专项验收。

## 常见问题

### Add New Panel 中没有插件

先在启动 RViz 的终端检查：

```bash
ros2 pkg prefix joint_state_rviz_panel
```

应返回本工作空间的安装路径。若找不到，重新加载 `install/setup.bash`；若 RViz 已经运行，需重新启动。

插件索引位于包安装前缀下的：

```text
share/ament_index/resource_index/rviz_common__pluginlib__plugin/joint_state_rviz_panel
```

### 插件加载失败或提示 undefined symbol

确认使用 Qt5 与 ROS 2 Humble，重新编译并加载同一个工作空间。可检查动态库依赖：

```bash
ldd -r "$(ros2 pkg prefix joint_state_rviz_panel)/lib/libjoint_state_rviz_panel.so"
```

### 有发布者，但一直等待数据

```bash
ros2 topic info /joint_states -v
ros2 topic echo /joint_states --once
```

检查消息类型、名称/位置数组和 QoS。Best Effort 发布者与 Reliable 订阅者不兼容；默认使用 Best Effort 更适合传感器状态。

### 角度显示 raw

缺少关节类型配置。通过 YAML 指定 `revolute / continuous`，或明确启用未知类型旋转推断。不要将直线关节误作旋转关节。

### 没有发布者或跨机器收不到数据

检查话题命名空间、`ROS_DOMAIN_ID`、DDS 发现配置和网络。先确认 `ros2 topic echo` 能接收，再排查界面。

### RViz 本身闪退

保存启动终端日志和崩溃栈，区分面板、MoveIt、Qt 与图形驱动问题。面板列表缺失通常属于环境/索引问题，不能据此判断图形崩溃原因。

## 源码结构

```text
.
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml
├── README.md
├── config/monitor.yaml
├── rviz/joint_monitor.rviz
├── include/joint_state_rviz_panel/
├── src/
└── test/
```

本版本不包含运动控制、URDF 自动解析、历史曲线或数据库。

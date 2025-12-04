# 3自由度机械臂描述包

## 概述

这是一个ROS 2功能包，包含了一个3自由度机械臂的URDF描述、启动文件和RViz可视化配置。

## 机械臂结构

该机械臂具有3个旋转关节：
- 肩关节：绕Z轴旋转（左右转动）
- 肘关节：绕Y轴旋转（上下摆动）
- 腕关节：绕Y轴旋转（上下摆动）

## 安装

### 前提条件

- ROS 2 Humble或更高版本
- colcon构建系统

### 安装步骤

1. 创建一个ROS 2工作空间（如果尚未创建）：
   ```bash
   mkdir -p ~/ros2_ws/src
   ```

2. 克隆该功能包到工作空间的src目录：
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

3. 构建工作空间：
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. 激活工作空间：
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## 使用

### 启动可视化

使用以下命令启动机械臂的可视化界面：

```bash
ros2 launch arm3dof_description display_arm.launch.py
```

这将启动：
1. 关节状态发布器（带滑块GUI）
2. 机器人状态发布器
3. RViz2（加载默认配置）

### 操作机械臂

在关节状态发布器GUI中，您可以通过拖动滑块来控制机械臂的各个关节：
- `shoulder_joint`：控制肩关节的旋转
- `elbow_joint`：控制肘关节的旋转
- `wrist_joint`：控制腕关节的旋转

## 功能包结构

```
arm3dof_description/
├── launch/
│   └── display_arm.launch.py    # 启动文件
├── rviz/
│   └── arm_rviz.rviz            # RViz配置文件
├── urdf/
│   └── arm3dof.urdf             # URDF模型文件
├── CMakeLists.txt               # CMake构建文件
├── LICENSE                      # 许可证文件
├── package.xml                  # 包信息文件
└── README.md                    # 本说明文件
```

## URDF模型详情

### 连杆（Links）

| 连杆名称 | 尺寸（长×宽×高） | 质量 | 颜色 |
|---------|----------------|------|------|
| base_link | 0.2×0.2×0.1 | 1.0 kg | 灰色 |
| link1 | 0.3×0.05×0.05 | 0.5 kg | 蓝色 |
| link2 | 0.3×0.05×0.05 | 0.3 kg | 蓝色 |
| link3 | 0.3×0.05×0.05 | 0.2 kg | 蓝色 |

### 关节（Joints）

| 关节名称 | 类型 | 轴 | 范围 | 速度限制 |
|---------|------|----|------|----------|
| shoulder_joint | revolute | Z轴 | ±90度 | 0.5 rad/s |
| elbow_joint | revolute | Y轴 | 0~135度 | 0.5 rad/s |
| wrist_joint | revolute | Y轴 | ±90度 | 0.5 rad/s |

## 自定义

### 修改URDF模型

您可以编辑`urdf/arm3dof.urdf`文件来修改机械臂的结构，例如：
- 调整连杆尺寸
- 修改关节范围
- 更改质量和惯性参数
- 添加新的连杆或关节

### 修改RViz配置

您可以在RViz中保存自定义配置，然后更新`rviz/arm_rviz.rviz`文件。

## 许可证

该功能包使用Apache 2.0许可证，详情请见LICENSE文件。

## 维护者

- dubhe <a18967707065@icloud.com>

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # 获取功能包路径
    pkg_dir = get_package_share_directory('arm3dof_description')
    # URDF文件路径
    urdf_path = os.path.join(pkg_dir, 'urdf', 'arm3dof.urdf')

    # 1. 声明URDF文件路径参数
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=urdf_path,
        description='Path to the URDF file'
    )

    # 2. 从文件加载URDF内容
    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf_file')]),
        value_type=str
    )

    # 3. 关节状态发布器（带滑块GUI）
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}]
    )

    # 4. 机器人状态发布器
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 5. 启动RViz2（加载默认配置）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'arm_rviz.rviz')]
    )

    return LaunchDescription([
        urdf_file_arg,
        joint_state_gui,
        robot_state_pub,
        rviz
    ])
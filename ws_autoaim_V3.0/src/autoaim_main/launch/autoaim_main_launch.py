from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类- ----- ---
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取--------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关----------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关-----------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关-----------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os
# 参数描述类
from launch_ros.parameter_descriptions import ParameterValue
# 指令执行类
from launch.substitutions import Command


def generate_launch_description():
    autoaim_main = get_package_share_directory('autoaim_main')
    rviz_config_dir =os.path.join(autoaim_main,'rviz','rviz.rviz')
    # 启动rviz2
    rviz_node =Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # 加载模型
    model=DeclareLaunchArgument(name="model",default_value=get_package_share_directory("state_prediction")+"/urdf/autoaim_description.urdf")
    p_value=ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))
    robot_state_pub=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":p_value}]
    )

    # 动态模型发布节点
    joint_state_pub=Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )

    # 相机节点
    hik_camera_launch=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("hik_camera"),"launch","hik_camera.launch.py"
            )
        )
    )

    # 装甲板检测分类节点
    armor_detect_node = Node(
        package="cv_processing",
        executable="detector_node"
    )

    # 串口节点
    autoaim_serial_node = Node(
        package="autoaim_serial",
        executable="autoaim_serial_driver"
    )

    # 装甲板坐标转换节点
    coordinate_transfor_node = Node(
        package="state_prediction",
        executable="coordinate_transfor"
    )

    # 弹道解析节点
    ballistic_calculating_node = Node(
        package="ballistic_calculating",
        executable="ballistic_calculating_node"
    )

    # 可视化调试节点
    debug_show_node = Node(
        package="autoaim_debug",
        executable="debug_node"
    )


    return LaunchDescription([
        model,
        robot_state_pub,
        joint_state_pub,  
        armor_detect_node,
        coordinate_transfor_node,
        ballistic_calculating_node,
        debug_show_node,
        hik_camera_launch,
        autoaim_serial_node,
        rviz_node
    ])
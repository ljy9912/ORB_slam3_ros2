from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义节点配置
    calibration_node = Node(
        package='calibration',          # 包名
        executable='joint_calibration_node',  # 可执行文件名（即节点名）
        name='joint_calibration',       # 节点运行时名称（可选，默认与executable相同）
        output='screen',                # 输出日志到屏幕
        parameters=[{'debug_mode': True}]  # 传递参数示例（按需修改）
    )

    return LaunchDescription([
        calibration_node
    ])
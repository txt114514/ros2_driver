from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # 声明可配置参数（可在命令行覆盖）
    error_value_xy_arg = DeclareLaunchArgument(
        'error_value_xy',
        default_value='0.1',
        description='Position tolerance threshold'
    )

    error_value_yaw_arg = DeclareLaunchArgument(
        'error_value_yaw',
        default_value='0.1',
        description='Yaw tolerance threshold'
    )

    # 创建节点
    doing_exec_node = Node(
        package='execution',          # ⚠ 改成你的包名
        executable='doing_exec.py',              # ⚠ 改成你的入口名
        name='doing_exec',
        output='screen',
        parameters=[{
            'error_value_xy': LaunchConfiguration('error_value_xy'),
            'error_value_yaw': LaunchConfiguration('error_value_yaw'),
        }]
    )

    return LaunchDescription([
        error_value_xy_arg,
        error_value_yaw_arg,
        doing_exec_node
    ])
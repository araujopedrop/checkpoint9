from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch
from launch_ros.actions import Node


def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.5"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="0"
    )

    obstacle_ = LaunchConfiguration('obstacle')
    degrees_ = LaunchConfiguration('degrees')

    pre_approach_node = Node(
        package='attach_shelf',
        executable='pre_approach_node',
        output='screen',
        name='pre_approach_node',
        emulate_tty=True,
        parameters=[
            {"obstacle": obstacle_},{"degrees": degrees_},]
    )

    message_info = launch.actions.LogInfo(
        msg=str(obstacle_))

    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            message_info,
            pre_approach_node
        ]
    )
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
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false"
    )

    obstacle_ = LaunchConfiguration('obstacle')
    degrees_ = LaunchConfiguration('degrees')
    final_approach_ = LaunchConfiguration('final_approach')

    pre_approach_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2_node',
        emulate_tty=True,
        parameters=[
            {"obstacle": obstacle_},{"degrees": degrees_},{"final_approach": final_approach_},]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            pre_approach_node
        ]
    )
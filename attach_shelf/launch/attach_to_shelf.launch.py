import os
import launch
from launch_ros.actions          import Node
from launch                      import LaunchDescription
from launch.substitutions        import LaunchConfiguration
from launch.actions              import DeclareLaunchArgument
from ament_index_python.packages import get_package_prefix,get_package_share_directory

def generate_launch_description():

    package_description = "attach_shelf"

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

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_robot_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot_odom'])

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), "config", 'rviz_config.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            static_tf_pub,
            pre_approach_node,
            rviz_node
        ]
    )
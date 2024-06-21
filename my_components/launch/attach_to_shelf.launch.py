import launch
from launch_ros.actions          import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='PreApproach'),
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachServer',
                    name='AttachServer')
            ],
            output='screen',
    )

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_robot_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'])


    return launch.LaunchDescription([container,static_tf_pub])
import launch
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description with multiple components."""
    
    package = 'my_components'

    # Full path to an RViz2 .rviz config
    rviz_config = PathJoinSubstitution([FindPackageShare(package), "rviz", "config.rviz"])
    
    # --- RViz2 using the config arg ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config])

    # --- Composable Node Container with 2 components
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # multithreaded
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach',
                    parameters=[{
                        "exit_on_end": False # prevent Application closing
                    }]),
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachServer',
                    name='attach_server',
                    parameters=[{'use_sim_time': True}])
            ],
            output='screen',
    )
    
    # Delay Container by 5s 
    delayed_container= TimerAction(
        period=5.0,  # Seconds to wait
        actions=[container]
    )   


    return launch.LaunchDescription([
    rviz_node,
    delayed_container])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_description = "attach_shelf"

    # -- launch args --
    obstacl_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.5", description="Distance (in meters) to stop before obstacle"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="0", description="Rotation degrees after stopping"
    )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false", description="Do final attachment to shelf after approach."
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([FindPackageShare(package_description), "rviz", "configv2.rviz"]),
        description="Full path to an RViz2 .rviz config",
    )

    # --- launch config handles ---
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')
    rviz_config = LaunchConfiguration('rviz_config')


    # --- RViz2 using the config arg ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config])

    # -- Service Server Node --
    service_node = Node(
        package=package_description,
        executable="approach_service",
        output="screen",
        name="approach_service_node",
        emulate_tty=True,
    )

    # -- Pre Approach V2 Node ---
    pre_approach_node = Node(
        package=package_description,
        executable="pre_approach_v2",
        output="screen",
        name="pre_approach_node",
        emulate_tty=True,
        parameters=[{
            "obstacle": obstacle,
            "degrees": degrees,
            "final_approach": final_approach
        }])
    
    # Delay Preapproach by 5s (adjust period as needed)
    delayed_preapproach= TimerAction(
        period=5.0,  # Seconds to wait
        actions=[pre_approach_node]
    )

    return LaunchDescription([
        obstacl_arg,
        degrees_arg,
        final_approach_arg,
        rviz_config_arg,
        rviz_node,
        service_node,
        delayed_preapproach
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    fastdds_env = {
        'RMW_IMPLEMENTATION':   'rmw_fastrtps_cpp',
        'ROS_DISCOVERY_SERVER': '192.168.1.18:11811',
        'ROS_SUPER_CLIENT':     'True',
        'ROS_DOMAIN_ID':        '0',
    }

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 visualiser')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')


    # SLAM
    slam_toolbox_node = Node(
        package = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name = 'slam_toolbox',
        output = 'screen',
        additional_env = fastdds_env,
    )

    # Frontier Explorer
    frontier_explorer_node = Node(
        package = 'frontier_exploration_mapping',
        executable = 'frontier_explorer_node',
        name = 'frontier_explorer',
        output = 'screen',
        additional_env = fastdds_env,
        parameters = [{
            'use_sim_time':     use_sim_time,
            'min_cluster_size': 10,
            'alpha':            1.0,
            'beta':             2.0,
            'epsilon':          1e-6,
            'radius_meters':    0.5,
        }],
    )

    # Semantic Hazard Classifier
    semantic_classifier_node = Node(
        package = 'frontier_exploration_mapping',
        executable = 'semantic_hazard_classifier_node',
        name = 'semantic_hazard_classifier',
        output = 'screen',
        additional_env = fastdds_env,
    )

    # Behavior Coordinator
    behavior_coordinator_node = Node(
        package = 'frontier_exploration_mapping',
        executable = 'behavior_coordinator_node',
        name = 'behavior_coordinator',
        output = 'screen',
        additional_env = fastdds_env,
    )

    # Rviz2
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        condition = IfCondition(use_rviz),
        additional_env = fastdds_env,
    )


    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,

        slam_toolbox_node,

        frontier_explorer_node,
        semantic_classifier_node,
        behavior_coordinator_node,

        rviz_node,
    ])
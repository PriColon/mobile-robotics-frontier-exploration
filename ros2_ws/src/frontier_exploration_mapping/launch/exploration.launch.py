from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (true = sim, false = real robot)')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualiser')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')

    # ── Scan Relay (fixes sim frame_id issue) ─────────────────────────────
    scan_relay_node = Node(
        package    = 'frontier_exploration_mapping',
        executable = 'scan_relay_node',
        name       = 'scan_relay',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )

    # ── SLAM ──────────────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Navigation Planner ─────────────────────────────────────────────────
    navigation_planner_node = Node(
        package    = 'frontier_exploration_mapping',
        executable = 'navigation_planner_node',
        name       = 'navigation_planner',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )

    # ── Frontier Explorer ──────────────────────────────────────────────────
    frontier_explorer_node = Node(
        package    = 'frontier_exploration_mapping',
        executable = 'frontier_explorer_node',
        name       = 'frontier_explorer',
        output     = 'screen',
        parameters = [{
            'use_sim_time':     use_sim_time,
            'min_cluster_size': 10,
            'alpha':            1.0,
            'beta':             2.0,
            'epsilon':          1e-6,
            'radius_meters':    0.5,
        }],
    )

    # ── Semantic Hazard Classifier ─────────────────────────────────────────
    semantic_classifier_node = Node(
        package    = 'frontier_exploration_mapping',
        executable = 'semantic_hazard_classifier_node',
        name       = 'semantic_hazard_classifier',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )

    # ── Behavior Coordinator ───────────────────────────────────────────────
    behavior_coordinator_node = Node(
        package    = 'frontier_exploration_mapping',
        executable = 'behavior_coordinator_node',
        name       = 'behavior_coordinator',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )

    # ── RViz2 ──────────────────────────────────────────────────────────────
    rviz_node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        condition  = IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,

        scan_relay_node,
        slam_launch,
        navigation_planner_node,
        frontier_explorer_node,
        semantic_classifier_node,
        behavior_coordinator_node,
        rviz_node,
    ])

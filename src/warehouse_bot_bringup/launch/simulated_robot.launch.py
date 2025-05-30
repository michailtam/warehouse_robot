import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("use_slam")
    use_rviz = LaunchConfiguration("use_rviz")
    slam_config = LaunchConfiguration("slam_config")

    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("warehouse_bot_navigation"), "config", "slam_toolbox.yaml"),
        description="Full path to slam yaml file to load"
    )

    # Launch Gazebo simulator
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_bot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    # Launch the controllers
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_bot_controller"),
            "launch",
            "controller.launch.py"
        )
    )

    # Start the Localization process
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_bot_navigation"),
            "launch",
            "localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    # Launch SLAM
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_slam)
    )

    # Save the map to hard disk when SLAM is desired
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65}
        ],
        condition=IfCondition(use_slam)
    )

    # Start the SLAM process for map creation automatically without doing it manually
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": ["map_saver_server", "slam_toolbox"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
        condition=IfCondition(use_slam)
    )

    # Improve the Localization with an Extended Kalman Filter (EKF) 
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("warehouse_bot_navigation"), "config", "ekf.yaml")],
    )

    # Launch rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("warehouse_bot_navigation"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_slam_arg,
        use_rviz_arg,
        slam_config_arg,
        gazebo,
        controller,
        localization,
        slam,
        ekf,
        nav2_map_saver,
        nav2_lifecycle_manager,
        rviz
    ])
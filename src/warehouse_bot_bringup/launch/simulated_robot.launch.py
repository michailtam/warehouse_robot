import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
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

    lifecycle_nodes = ["map_saver_server", "slam_toolbox"]

    # nav2_lifecycle_manager = Node(
    #     package="nav2_lifecycle_manager",
    #     executable="lifecycle_manager",
    #     name="lifecycle_manager_slam",
    #     output="screen",
    #     parameters=[
    #         {"node_names": lifecycle_nodes},
    #         {"use_sim_time": use_sim_time},
    #         {"autostart": True}
    #     ],
    # )

    # Launch robot localization
    robot_localization = Node(
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
        # nav2_lifecycle_manager,
        gazebo,
        controller,
        slam,
        robot_localization,
        rviz
    ])
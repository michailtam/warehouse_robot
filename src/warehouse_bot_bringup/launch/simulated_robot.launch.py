import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")
    use_rviz = LaunchConfiguration("use_rviz")

    # Declare the launch arguments
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true"
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

    # Start rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_rviz_arg,
        gazebo,
        controller,
        rviz
    ])
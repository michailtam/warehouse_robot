import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    warehouse_bot_description = get_package_share_directory("warehouse_bot_description")

    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Set default launch arguments
    declare_x_pos_arg = DeclareLaunchArgument(
        name='x',
        default_value='1.0',
        description='x-position')

    declare_y_pos_arg = DeclareLaunchArgument(
        name='y',
        default_value='7.0',
        description='y-position')
    
    declare_z_pos_arg = DeclareLaunchArgument(
        name='z',
        default_value='0.0',
        description='z-position')

    declare_roll_arg = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation in radians')

    declare_pitch_arg = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation in radians')

    declare_yaw_arg = DeclareLaunchArgument(
        name='yaw',
        default_value='-1.57',
        description='yaw angle of initial orientation in radians')

    # Declare the launch arguments
    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(warehouse_bot_description, "urdf", "warehouse_bot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    
    # Retrieve the world to use based of the world_name
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="warehouse")
    world_path = PathJoinSubstitution([
            warehouse_bot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    # Set the path containing the models
    model_path = str(Path(warehouse_bot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("warehouse_bot_description"), 'models')
    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    # Get the robot description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    # Execute the robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Launch the desired world in Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "warehouse_bot",
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ]
    )

    # Connect Gazebo with ROS2
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )

    return LaunchDescription([
        model_arg,
        declare_x_pos_arg,
        declare_y_pos_arg,
        declare_z_pos_arg,
        declare_roll_arg,
        declare_pitch_arg,
        declare_yaw_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Start the controllers i.e. joint state broadcaster and the controller for the wheels
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Start the controllers for the wheels
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["warehouse_bot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ]
    )

    # Transforms the Twist message to TwistStamped message
    twist_to_stamped_node = Node(
        package='warehouse_bot_controller',
        executable='twist_relay.py',
        name='twist_relay',
        output='screen'
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            twist_to_stamped_node
        ]
    )
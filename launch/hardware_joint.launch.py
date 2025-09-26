from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Visualize the robot in RViz'
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('so_100_arm'), 'config', 'so_100_arm_wheel.urdf.xacro']
                ),
                ' ',
                'use_fake_hardware:=false'
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, # Pass the robot description to the controller manager
            PathJoinSubstitution(
                [FindPackageShare('so_100_arm'), 'config', 'controllers.yaml']
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so_100_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    track_and_arm_controller_node = Node(
        package="so_100_track",
        executable="controller",
        name="track_and_arm_controller",
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('so_100_arm'), 'config', 'urdf.rviz'])],
        output='screen'
    )

    nodes = [
        controller_manager,
        # robot_state_pub_node,
        # joint_state_publisher_node, # Add the aggregator
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        # track_and_arm_controller_node, # Add your custom node
        rviz_node,
    ]

    return LaunchDescription([rviz_arg] + nodes)
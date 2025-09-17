from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # Add launch argument
    zero_pose_arg = DeclareLaunchArgument(
        'zero_pose',
        default_value='false',
        description='Test zero pose after startup'
    )

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
    joint_state_pub = Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui', 
                parameters=[
                    # PathJoinSubstitution(
                    #     [FindPackageShare('so_100_arm'), 'config', 'initial_positions_launch.yaml']
                    # ),
                    {"zeros.Elbow": -1.6, "zeros.Gripper": -1.5, "zeros.Shoulder_Pitch": 1.5, "zeros.Shoulder_Rotation": 0, "zeros.Wrist_Pitch": -0.1, "zeros.Wrist_Roll": 1.2, }
                ],
        remappings=[
            ('/joint_states', '/so101track/joint_states'),
        ]
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('so_100_arm'), 'config', 'controllers.yaml']
            ),
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after joint_state_broadcaster
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so_100_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    joint_state_controller_spawner = Node(
        package="so_100_track",
        executable="controller",
        name="joint_state_controller",
    )

    # Delay loading and starting robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('so_100_arm'), 'config', 'urdf.rviz'])]
    )

    # Add zero pose test node
    zero_pose_node = Node(
        condition=IfCondition(LaunchConfiguration('zero_pose')),
        package='so_arm_100_hardware',
        executable='zero_pose.py',
        name='zero_pose_test',
    )

    nodes = [
        robot_state_pub_node,
        # joint_state_pub,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        rviz_node,
        zero_pose_node,
        # joint_state_controller_spawner
    ]

    return LaunchDescription([zero_pose_arg, rviz_arg] + nodes) 

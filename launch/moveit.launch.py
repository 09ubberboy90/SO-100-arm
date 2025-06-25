from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware'
    )
    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('so_100_arm'), 'config', 'so_100_arm.urdf.xacro']
                ),
                ' ',
                'use_fake_hardware:=false'
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/robot_description', '/controller_manager/robot_description'),
        ]
    )

    moveit_config = MoveItConfigsBuilder("so_100_arm", package_name="so_100_arm").to_moveit_configs()
    # Add the use_fake_hardware parameter
    moveit_config.robot_description = {
        "use_fake_hardware": LaunchConfiguration('use_fake_hardware')
    }
    

    return LaunchDescription([
        use_fake_hardware_arg,
        robot_state_pub_node,
        generate_demo_launch(moveit_config)
    ])

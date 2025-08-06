from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Locate package
    pkg_share = FindPackageShare('fairino3_v6_moveit2_config')

    # File paths
    urdf_file = PathJoinSubstitution([pkg_share, 'config', 'fairino3_v6_robot.urdf.xacro'])
    ros2_controllers = PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])
    srdf_file = PathJoinSubstitution([pkg_share, 'config', 'fairino3_v6_robot.srdf'])
    kinematics_yaml = PathJoinSubstitution([pkg_share, 'config', 'kinematics.yaml'])
    ompl_yaml = PathJoinSubstitution([pkg_share, 'config', 'ompl_planning.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'moveit.rviz'])

    # Generate robot_description via xacro (real hardware)
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Generate robot_description_semantic via cat
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command(['cat ', srdf_file]),
            value_type=str
        )
    }

    # Start robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Start ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers],
        output='screen'
    )

    # Load controllers (joint state broadcaster and arm controller)
    load_controllers = [
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', ctrl],
            output='screen'
        ) for ctrl in ['joint_state_broadcaster', 'arm_controller']
    ]

    # Start Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            ros2_controllers,
            {'moveit_controller_manager': 'moveit_ros_control_interface::MoveItRosControlControllerManager'}
        ]
    )

    # Start RViz with both descriptions
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[robot_description, robot_description_semantic]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        *load_controllers,
        move_group_node,
        rviz_node
    ])

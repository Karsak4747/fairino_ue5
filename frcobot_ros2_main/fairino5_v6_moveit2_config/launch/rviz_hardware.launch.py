from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Locate package
    pkg_share = FindPackageShare('fairino5_v6_moveit2_config')

    # File paths
    urdf_file = PathJoinSubstitution([pkg_share, 'config', 'fairino5_v6_robot.urdf.xacro'])
    ros2_controllers = PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])
    srdf_file = PathJoinSubstitution([pkg_share, 'config', 'fairino5_v6_robot.srdf'])
    kinematics_yaml = PathJoinSubstitution([pkg_share, 'config', 'kinematics.yaml'])
    ompl_yaml = PathJoinSubstitution([pkg_share, 'config', 'ompl_planning.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'moveit.rviz'])

    # Generate robot_description via xacro
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Generate robot_description_semantic
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
        name='controller_manager',
        output='screen',
        parameters=[robot_description, ros2_controllers],
    )

    # Spawn controllers
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_fairino5_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino5_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delayed_spawn_jsb = TimerAction(period=2.0, actions=[spawn_joint_state_broadcaster])
    delayed_spawn_arm = TimerAction(period=4.0, actions=[spawn_fairino5_controller])

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

    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[robot_description, robot_description_semantic]
    )

    # --- Marker publisher (STL) ---
    # путь до скрипта в твоём пакете
    stl_script = os.path.join(
        get_package_share_directory('fairino5_v6_moveit2_config'),
        'config',
        'stl_marker_pub.py'
    )

    stl_marker_proc = ExecuteProcess(
        cmd=['python3', stl_script],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        delayed_spawn_jsb,
        delayed_spawn_arm,
        move_group_node,
        rviz_node,
        stl_marker_proc  # добавили публикацию STL
    ])

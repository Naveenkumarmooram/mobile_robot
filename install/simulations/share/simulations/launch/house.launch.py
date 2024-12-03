# from launch.actions import SetEnvironmentVariable, TimerAction
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from ament_index_python.packages import get_package_share_directory
# import os
# import xacro
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():
#     # Paths to launch files and URDF
#     launch_file_dir = os.path.join(get_package_share_directory('simulations'), 'launch')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     # ROS 2 launch configuration substitutions
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     x_pose = LaunchConfiguration('x_pose', default='-2.0')
#     y_pose = LaunchConfiguration('y_pose', default='-0.5')

#     # Path to the world file
#     world = os.path.join(
#         get_package_share_directory('worlds'),
#         'worlds',
#         'house.world'  # Assuming you're using this world file
#     )

#     # Path to the URDF or Xacro file
#     robot_description_path = os.path.join(
#         get_package_share_directory('robot_description'),
#         'urdf',
#         'robot_base.urdf'  # Replace with your actual robot URDF file name
#     )

#     # Process Xacro if needed
#     doc = xacro.process_file(robot_description_path)
#     robot_description = doc.toxml()

#     # Include gzserver launch description
#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     # Include gzclient launch description
#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     # Robot state publisher node
#     robot_state_publisher_cmd = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='log',
#         parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
#     )

#     # Joint state publisher node
#     joint_state_publisher_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     # Spawn robot node using ros2 service
#     spawn_robot_cmd = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',  # If not available, consider using the service call as described
#         arguments=['-entity', 'mobile_robot', '-file', robot_description_path, '-x', x_pose, '-y', y_pose, '-z', '0.0'],
#         output='screen'
#     )

#     # Create and return the LaunchDescription object
#     ld = LaunchDescription()

#     # Add all the nodes/actions to the launch description
#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(robot_state_publisher_cmd)
#     ld.add_action(spawn_robot_cmd)
#     ld.add_action(joint_state_publisher_node)

#     return ld
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get the package directories
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_worlds = get_package_share_directory('worlds')
    urdf_file_path = os.path.join(pkg_robot_description, 'urdf', 'robot_base.urdf')
    gazebo_models = get_package_share_directory('worlds')

    # Process the robot URDF file with xacro
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_urdf = robot_description_config.toxml()

    # Nodes for publishing robot state and joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Set up Gazebo server and client with the world file
    world_file_name = 'cafe.world'  # Make sure this file exists in the 'worlds' package
    world_path = os.path.join(pkg_worlds, 'worlds', world_file_name)
    
    # Gazebo server command with world file argument
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world_path}.items()  # Ensure the world file path is correct
    )

    # Gazebo client command to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Delay robot spawning by 5 seconds to ensure Gazebo is ready
    urdf_spawn_node = TimerAction(
        period=5.0,  # Delay before spawning
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mobile_robot', '-file', urdf_file_path, '-x', '0', '-y', '0', '-z', '0'],
            output='screen'
        )]
    )

    # Set the path to the SDF model files
    gazebo_models_path = os.path.join(gazebo_models, 'models')
    
    # Set the environment variable for Gazebo model path
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', gazebo_models_path
    )

    return LaunchDescription([
        set_gazebo_model_path,   # Include the environment variable
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])

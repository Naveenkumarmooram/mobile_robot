from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_worlds = get_package_share_directory('worlds')

    # Process the URDF file
    urdf_file_path = os.path.join(pkg_robot_description, 'urdf', 'robot_base.urdf')
    with open(urdf_file_path, 'r') as file:
        robot_description = file.read()

    # Set up the world file path
    world_file_path = os.path.join(pkg_worlds, 'worlds', 'empty_world.world')

    # Include the Gazebo launch file with necessary plugins
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --remap /tf:=tf /tf_static:=tf_static'
        }.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'mobile_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Set the environment variable for Gazebo model path
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', os.path.join(pkg_worlds, 'models')
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the paths to the required packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description')

    # Set the path to the world file
    world_file_path = '/mobile_robot/src/worlds/worlds/empty_world.world'

    # Read the URDF file for the robot description
    urdf_file_path = os.path.join(pkg_robot_description, 'urdf', 'robot_base.urdf')
    with open(urdf_file_path, 'r') as file:
        robot_description = file.read()

    # Set environment variables to specify Gazebo paths for resources and models
    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', '/mobile_robot/src/worlds/worlds'
    )
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', '/mobile_robot/src/worlds/models'
    )
    set_shader_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}'
    )

    # Include the Gazebo launch file with the necessary arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --remap /tf:=tf /tf_static:=tf_static -s libgazebo_ros_factory.so'
        }.items()
    )

    # Node for the robot state publisher to publish the robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Node to spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'mobile_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Return the LaunchDescription with all components
    return LaunchDescription([
        set_gazebo_resource_path,
        set_gazebo_model_path,
        set_shader_path,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])

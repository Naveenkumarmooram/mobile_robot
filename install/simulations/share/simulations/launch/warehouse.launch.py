import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get paths for the world and robot description
    worlds_share = get_package_share_directory('worlds')
    robot_description_share = get_package_share_directory('robot_description')

    warehouse_world_path = os.path.join(worlds_share, 'models', 'aws_robomaker_warehouse_ClutteringA_01', 'model.sdf')
    robot_description_path = os.path.join(robot_description_share, 'urdf', 'robot_base.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', warehouse_world_path],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-file', robot_description_path, 
                       '-entity', 'warehouse_robot',
                       '-x', '0', '-y', '0', '-z', '0']
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': Command(['xacro ', robot_description_path])}]
        ),
    ])
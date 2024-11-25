from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare('smart_car').find('smart_car')
    
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'smartcar.urdf'])
    default_world_path = PathJoinSubstitution([pkg_share, 'world', 'smalltown.world'])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Path to smartcar URDF file'),
        DeclareLaunchArgument(name='world', default_value=default_world_path, description='Path to Gazebo world file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Path to rviz config file'),

        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'extra_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
            }.items()
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='smart_car',
            executable='joint_state_publisher.py',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='smart_car',
            executable='wheel_odometry.py',
            name='wheel_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_smartcar',
            output='screen',
            arguments=[LaunchConfiguration('model')],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'smartcar', '-file', LaunchConfiguration('model')],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        )
    ])
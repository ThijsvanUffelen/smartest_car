from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    
    pkg_share = FindPackageShare('smart_car').find('smart_car')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'smartcar.urdf'])
    default_world_path = PathJoinSubstitution([pkg_share, 'world', 'smalltown.world'])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_share, 'map', 'smalltown_world.yaml']),
        description='Full path to map yaml file to load')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_share, 'config', 'nav2_parameters.yaml']),
        description='Full path to the ROS2 parameters file')
        
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Path to smartcar URDF file')
        
    declare_world_path_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to Gazebo world file')
        
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Path to rviz config file')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'extra_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        }.items()
    )

    joint_state_publisher_cmd = Node(
        package='smart_car',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    wheel_odometry_cmd = Node(
        package='smart_car',
        executable='wheel_odometry.py',
        name='wheel_odometry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        output='screen',
        arguments=[LaunchConfiguration('model')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'smartcar', '-file', LaunchConfiguration('model')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'yaml_filename': map_yaml_file}])
                   
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': True},
                   {'node_names': ['map_server',
                                 'amcl',
                                 'controller_server',
                                 'planner_server',
                                 'bt_navigator']}])

    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file])

    start_planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file])

    start_controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file])

    start_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file])

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    
    ld.add_action(gazebo_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(wheel_odometry_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(ekf_node_cmd)
    
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_planner_server_cmd)
    ld.add_action(start_controller_server_cmd)
    ld.add_action(start_bt_navigator_cmd)
    
    return ld

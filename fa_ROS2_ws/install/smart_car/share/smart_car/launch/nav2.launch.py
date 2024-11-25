from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    
    pkg_share = FindPackageShare('smart_car')
    
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
                                 'recoveries_server',
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

    start_recoveries_server_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
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
    
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_planner_server_cmd)
    ld.add_action(start_controller_server_cmd)
    ld.add_action(start_bt_navigator_cmd)
    ld.add_action(start_recoveries_server_cmd)
    
    return ld

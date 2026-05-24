from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    config_files_pkg_path= get_package_share_directory('config_files')
    ros_gz_sim_pkg_path  = get_package_share_directory('ros_gz_sim')
    gazebo_envs_pkg_path = get_package_share_directory('gazebo_envs')
    gz_sim_launch_path   = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    gz_spawn_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_spawn_model.launch.py'])
    world_file_path      = PathJoinSubstitution([gazebo_envs_pkg_path, 'worlds', 'simple_house.world'])

    description_pkg_path  = get_package_share_directory('justina_description')
    xacro_file_path = os.path.join(description_pkg_path, 'urdf','justina.xacro')
    robot_description_content = xacro.process_file(xacro_file_path).toxml()
       
    gz_bridge_params_path = os.path.join(gazebo_envs_pkg_path, 'config', 'gz_bridge.yaml')
    rviz_config_file = os.path.join(config_files_pkg_path, 'rviz', 'simple_house.rviz')
    map_config_file = os.path.join(config_files_pkg_path, 'navigation', 'simple_house.yaml')

    xarm_descrip_pkg_path = get_package_share_directory('xarm_description')
    xarm_controller_pkg_path = get_package_share_directory('xarm_controller')
    xarm_controller_file = PathJoinSubstitution([xarm_controller_pkg_path, 'config', 'xarm6_controllers.yaml'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(gazebo_envs_pkg_path, 'models') + ":" + os.path.join(xarm_descrip_pkg_path, "..")
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch_path),
            launch_arguments={
                'gz_args':[f'-r ', world_file_path],
                'on_exit_shutdown':'True',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_spawn_launch_path),
            launch_arguments={
                'world':'default',
                'topic':'/robot_description',
                'entity_name': 'justina',
                'x': '-3.5',
                'y': '0.0',
                'z': '1.0',
                'Y': '1.5708'
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p',
                f'config_file:={gz_bridge_params_path}',
                '-p', 'use_sim_time:=True',
            ],
            output='screen'
        ),
        Node(
            name='justina_gui',
            package='justina_gui',
            executable='justina_gui_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file,'--ros-args', '-p', 'use_sim_time:=True',],
        ),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename':map_config_file}, {'use_sim_time':True}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'base_frame_id':'base_link'},
                {'set_initial_pose':True},
                {'use_sim_time':True},
                {'alpha1':0.01},
                {'alpha2':0.01},
                {'alpha3':0.1},
                {'alpha4':0.1}
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_util',
                    executable='lifecycle_bringup',
                    name='lifecycle_bringup',
                    output='screen',
                    arguments=['map_server', 'amcl']
                )
            ]
        )
    ])

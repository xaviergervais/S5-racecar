from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    # Declare launch arguments
    prefix = LaunchConfiguration('prefix').perform(context)
    problematique = LaunchConfiguration('problematique').perform(context)
    navigation = LaunchConfiguration('navigation').perform(context)

    # Define nodes for behaviors
    path_following_node = Node(
        package='racecar_behaviors',
        executable='path_following',
        name='path_following',
        output='screen',
        parameters=[{'max_speed': 1}],
        remappings=[('/cmd_vel', f'/{prefix}/cmd_vel_abtr_5'), ('/scan', f'/{prefix}/scan'), (f'/{prefix}/odom', f'/{prefix}/odometry/filtered')]
    )

    obstacle_detector_node = Node(
        package='racecar_behaviors',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        remappings=[('/cmd_vel', f'/{prefix}/cmd_vel_abtr_1'), ('/scan', f'/{prefix}/scan')]
    )

    # Return nodes
    return [
        path_following_node,
        obstacle_detector_node,
        ]


    
def generate_launch_description():
    # Declare launch arguments
    prefix_arg = DeclareLaunchArgument('prefix', default_value='racecar')
    problematique_arg = DeclareLaunchArgument('problematique', default_value='false')
    navigation_arg = DeclareLaunchArgument('navigation', default_value='false')
    uturn_x_arg = DeclareLaunchArgument('uturn_x', default_value='999')
    uturn_y_arg = DeclareLaunchArgument('uturn_y', default_value='999')
    fixed_frame_id_arg = DeclareLaunchArgument('fixed_frame_id', default_value='odom')


    # Add nodes to launch description
    return LaunchDescription([
        prefix_arg,
        problematique_arg,
        navigation_arg,

        OpaqueFunction(function=launch_setup)
    ])


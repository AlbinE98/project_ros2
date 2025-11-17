from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Launch-time parameters
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    scale_m = LaunchConfiguration('scale_m')
    frame_id = LaunchConfiguration('frame_id')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        # Serial / node params
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('scale_m', default_value='1.0'),
        DeclareLaunchArgument('frame_id', default_value='map'),

        # RViz toggles
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='/home/albin/uwb.rviz'),

        # Your read-only node (wrapper strips bare --ros-args)
        Node(
            package='uwb_ro_bridge',
            executable='uwb_read_only_node_launch',
            output='screen',
            arguments=[
                '--port', port,
                '--baud', baud,
                '--scale-m', scale_m,
                '--frame-id', frame_id,
            ],
        ),

        # Optional RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])

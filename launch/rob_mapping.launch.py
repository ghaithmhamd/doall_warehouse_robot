import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
'''
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/doall/maps/my_room_map
'''

def generate_launch_description():
    # Arduino controller node
    arduino_node = Node(
        package='doall',
        executable='arduino_controller',
        output='screen'
    )

    xacro_file = os.path.join(get_package_share_directory('doall'),'model', 'robot.xacro')
    
    # Robot description
    robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        )

    # SLAM Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('doall'),
                'config',
                'slam_toolbox_mapping.yaml'
            )
        }.items()
    )

    # RPLidar launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )
    
    return LaunchDescription([
        arduino_node,
        robot_state_pub_node,
        rplidar_launch,
        slam_launch
    ])
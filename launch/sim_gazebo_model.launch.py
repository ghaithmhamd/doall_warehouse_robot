# to launch this file : (ros2 launch doall sim_gazebo_model.launch.py)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription #, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
#from launch.actions import TimerAction


def generate_launch_description():
    robotXacroName= 'doall_robot'
    namePackage= 'doall'
    modelFileRelativePath= 'model/robot.xacro'
    pathModelFile= os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    robotDescription= xacro.process_file(pathModelFile).toxml()
    #launch the world on gz sim :
    ## this is if you are using your own world model :
    ##(gz sim -v -v4 warehouse_ws/src/doall/world/scaled_warehouse_garage.sdf )
    gazebo_rosPackageLaunch= PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), 'world', 'warehouse_garage.sdf')
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': f'-r -v -v4 {pathWorldFile}',  
            'on_exit_shutdown': 'true'
        }.items()
    )
    '''
    ## this is if you are using an empty world model :
    gazebo_rosPackageLaunch= PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )
    gazeboLaunch= IncludeLaunchDescription(
        gazebo_rosPackageLaunch,   
        launch_arguments={
            'gz_args':['-v -v4 empty.sdf'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    '''

    # launch the Robot State Publisher Node
    nodeRobotStatePublisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                     'use_sim_time':True}]
    )
    #spawn the robot
    spawnModelNodeGazebo= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.063',
            '-Y', '0.0'
        ],
        output='screen',
    )


    # launch the ros_gz bridge :
    bridge_params= os.path.join(
        get_package_share_directory(namePackage),
        'config',
        'bridge_parameters.yaml'
    )     
    start_gazebo_ros_bridge_cmd= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
       

    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    return LaunchDescriptionObject
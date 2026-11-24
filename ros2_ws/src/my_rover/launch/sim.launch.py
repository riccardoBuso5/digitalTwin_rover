import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_rover')
    
    # Elabora il file Xacro dell'URDF
    robot_description = xacro.process_file(
        os.path.join(pkg_path, 'urdf', 'rover.urdf.xacro')
    ).toxml()

    # Percorso del mondo con gli ostacoli
    world_path = os.path.join(pkg_path, 'worlds', 'arena.sdf')

    # Avvia Gazebo Sim con il mondo personalizzato
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Pubblica le trasformazioni TF del robot
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Inserisce il rover nella simulazione
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_rover', '-z', '0.1'],
        output='screen'
    )

    # Ponte bidirezionale ROS 2 <-> Gazebo per comandi e sensori
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/front_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rear_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # Nodo C++ con l'algoritmo di Braitenberg
    controller = Node(
        package='my_rover',
        executable='rover_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    wokwi_bridge = Node(
        package='my_rover',
        executable='wokwi_bridge.py',
        name='wokwi_bridge',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        bridge,
        controller,
        wokwi_bridge
    ])
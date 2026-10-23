"""Local launch file; uses installed Humble/Gazebo Classic packages."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        raise RuntimeError('Use ROS_DOMAIN_ID=87 for this isolated simulation')
    root = Path(__file__).resolve().parent
    world = root/'sim'/'shelf_demo.world'
    if not world.exists():
        raise RuntimeError('Run python3 build_sim.py first')
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    urdf = (tb3/'urdf'/'turtlebot3_waffle_pi.urdf').read_text()
    model = tb3/'models'/'turtlebot3_waffle_pi'/'model.sdf'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch'/'gazebo.launch.py')),
                                 launch_arguments={'world':str(world),'verbose':'true'}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'use_sim_time':True,'robot_description':urdf}],output='screen'),
        Node(package='gazebo_ros',executable='spawn_entity.py',output='screen',
             arguments=['-entity','waffle_pi','-file',str(model),'-x','2.2','-y','0','-z','0.01',
                        '-Y','0','-timeout','120'])])

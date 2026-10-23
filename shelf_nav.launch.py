"""Nav2 with a map/registry generated for this world only; no RViz dependency."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        raise RuntimeError('Use ROS_DOMAIN_ID=87 for this isolated simulation')
    root = Path(__file__).resolve().parent
    tb3_nav = Path(get_package_share_directory('turtlebot3_navigation2'))
    candidates = list(tb3_nav.rglob('waffle_pi.yaml'))
    # Choose the Humble version if the installed package carries multiple distros.
    candidates.sort(key=lambda p: ('humble' not in p.parts,str(p)))
    if not candidates:
        raise RuntimeError('TurtleBot3 Waffle Pi Nav2 YAML not found')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2/'launch'/'bringup_launch.py')),
        launch_arguments=dict(map=str(root/'sim'/'shelf_demo.yaml'),
                              params_file=str(candidates[0]),use_sim_time='True',
                              autostart='True',use_composition='False').items())])

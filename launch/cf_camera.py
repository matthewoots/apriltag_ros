import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction

def generate_launch_description():

    name = "cf1"

    # load calibration
    calibration_yaml = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'calibration',
        name + '.yaml')
    
    with open(calibration_yaml, 'r') as ymlfile:
        calibration = yaml.safe_load(ymlfile)
    
    # load ip and port from crazyflie
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    calib_params = [calibration] + [crazyflies]

    return LaunchDescription([
        Node(
            package="apriltag_ros",
            executable="cf_opencv_publisher.py",
            name='cf_streamer',
            output="screen",
            parameters=calib_params
        ),
])
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
    
   # load ip and port from wifi.config
    wifi_yaml = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'wifi_setup.yaml')
    
    with open(wifi_yaml, 'r') as ymlfile:
        wifi = yaml.safe_load(ymlfile)

    calib_params = [calibration] + [wifi]

    return LaunchDescription([
        Node(
            package="apriltag_ros",
            executable="cf_opencv_publisher.py",
            name='cf_streamer',
            output="screen",
            parameters=calib_params
        ),
])
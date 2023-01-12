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
    
    # load apriltag
    cfg_yaml = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg', 'tags_36h11.yaml')

    with open(cfg_yaml, 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    calib_params = [calibration]
    cfg_params = [cfg]

    return LaunchDescription([
        Node(
            package="apriltag_ros",
            executable="cf_opencv_publisher.py",
            name='cf_streamer',
            output="screen",
            parameters=calib_params
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name='apriltag_node',
            output="screen",
            remappings=[
                ('image_rect', name + '/image'),
                ('camera_info', name + '/camera_info'),
            ],
            parameters=cfg_params
        ),
])
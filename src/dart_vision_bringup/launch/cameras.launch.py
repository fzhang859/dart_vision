from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    camera_driver_launch = PathJoinSubstitution([
        FindPackageShare("hik_camera_driver"),
        "launch",
        "multi_camera.launch.py",
    ])

    cameras_file = PathJoinSubstitution([
        FindPackageShare("dart_vision_bringup"),
        "config",
        "cameras.yaml",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_driver_launch),
            launch_arguments={
                "cameras_file": cameras_file,
            }.items(),
        )
    ])

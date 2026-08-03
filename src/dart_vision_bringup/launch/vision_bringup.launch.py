from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_profile = LaunchConfiguration("detector_profile")
    detector_namespace = LaunchConfiguration("detector_namespace")

    cameras_launch = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_bringup"),
            "launch",
            "cameras.launch.py",
        ]
    )

    detector_launch = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_bringup"),
            "launch",
            "detector.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_profile",
                default_value="highbay_workspace.yaml",
                description="Detector profile filename in config/detector",
            ),
            DeclareLaunchArgument(
                "detector_namespace",
                default_value="",
                description="Namespace shared by the camera topics and detector node",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cameras_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
                launch_arguments={
                    "detector_profile": detector_profile,
                    "namespace": detector_namespace,
                }.items(),
            ),
        ]
    )

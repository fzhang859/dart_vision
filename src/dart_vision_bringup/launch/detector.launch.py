from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_profile = LaunchConfiguration("detector_profile")
    namespace = LaunchConfiguration("namespace")

    detector_launch = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_detector"),
            "launch",
            "detector.launch.py",
        ]
    )

    detector_params = PathJoinSubstitution(
        [
            FindPackageShare("dart_vision_bringup"),
            "config",
            "detector",
            detector_profile,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_profile",
                default_value="highbay_workspace.yaml",
                description="Detector profile in bringup/config/detector",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Camera and detector namespace",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
                launch_arguments={
                    "params_file": detector_params,
                    "namespace": namespace,
                    "node_name": "detector_node",
                }.items(),
            ),
        ]
    )

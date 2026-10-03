"""Sobe o rosbridge (ponte WebSocket para o celular) e os dois nós deste pacote.

Uso:
    source /opt/ros/humble/setup.bash
    source install/setup.bash   # depois de um colcon build
    ros2 launch ponte_visao_drone sistema.launch.py
    ros2 launch ponte_visao_drone sistema.launch.py camera_index:=1   # outra webcam
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            get_package_share_directory("rosbridge_server") + "/launch/rosbridge_websocket_launch.xml"
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("camera_index", default_value="0", description="Índice da webcam (OpenCV)."),
        rosbridge_launch,
        Node(
            package="ponte_visao_drone",
            executable="piloto_gestos",
            name="piloto_gestos",
            output="screen",
            parameters=[{"camera_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int)}],
        ),
        Node(
            package="ponte_visao_drone",
            executable="visor_camera_drone",
            name="visor_camera_drone",
            output="screen",
        ),
    ])

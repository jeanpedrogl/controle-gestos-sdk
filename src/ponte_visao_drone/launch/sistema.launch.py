"""Sobe o rosbridge (ponte WebSocket para o celular) e o nó `piloto_gestos`.

O piloto já mostra o vídeo da câmera do drone (é de onde ele tira os gestos); o nó
`visor_camera_drone` continua existindo (`ros2 run ponte_visao_drone visor_camera_drone`), mas
não sobe junto para não abrir uma segunda janela decodificando o mesmo vídeo.

Uso:
    source /opt/ros/humble/setup.bash
    source install/setup.bash   # depois de um colcon build
    ros2 launch ponte_visao_drone sistema.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            get_package_share_directory("rosbridge_server") + "/launch/rosbridge_websocket_launch.xml"
        )
    )
    return LaunchDescription([
        rosbridge_launch,
        Node(
            package="ponte_visao_drone",
            executable="piloto_gestos",
            name="piloto_gestos",
            output="screen",
        ),
    ])

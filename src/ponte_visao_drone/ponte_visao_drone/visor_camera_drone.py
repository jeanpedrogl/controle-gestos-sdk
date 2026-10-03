"""Mostra no computador o vídeo da câmera do drone, retransmitido pelo celular.

O celular decodifica o vídeo real (OcuSync via DJI SDK) e publica frames
JPEG comprimidos no tópico `/phone/drone_camera/compressed`. Este nó só
assina e exibe — é puramente monitoramento, não controla nada (quem
controla o drone é a webcam, via `integracao_visao_drone.py`).
"""

import signal

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

JANELA = "Camera do drone (via celular)"


class VisorCameraDrone(Node):
    def __init__(self) -> None:
        super().__init__("visor_camera_drone")
        self.create_subscription(
            CompressedImage, "/phone/drone_camera/compressed", self._ao_receber, 1
        )
        self.get_logger().info("Aguardando vídeo em /phone/drone_camera/compressed...")

    def _ao_receber(self, msg: CompressedImage) -> None:
        dados = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(dados, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow(JANELA, frame)
            cv2.waitKey(1)


def main() -> None:
    rclpy.init()
    no = VisorCameraDrone()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        # Num Ctrl+C dentro do `ros2 launch` chegam dois SIGINT; ignora o segundo durante a limpeza.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

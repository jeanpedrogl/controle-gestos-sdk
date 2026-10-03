"""Traduz os gestos de `visao_mao.py` em comandos reais de voo para o drone.

Roda a webcam localmente, classifica o gesto a cada frame e publica os
comandos em tópicos ROS2. A ponte no celular (app Drone Bridge) assina esses
tópicos via rosbridge e repassa ao DJI SDK — nenhuma lógica de visão mora
aqui, isso fica inteiramente em `visao_mao.py`; a conversão gesto -> velocidade
(rampa, filtro de retorno da mão) fica em `comando_voo.py`.

Convenções de eixo (REP 103, padrão ROS): x = frente, y = esquerda, z = cima,
yaw positivo = sentido anti-horário (giro para a esquerda). O lado do celular
é quem converte isso para as unidades/sinais que o SDK da DJI espera.

Segurança (regras do `MANUAL_DESENVOLVEDOR.md` do Drone Bridge):
    - começa desarmado; só arma com a tecla 'a' e só com o celular conectado
      (telemetria chegando e o celular assinando /drone/stick_mode);
    - enquanto armado, publica /drone/cmd_vel a 20 Hz, mesmo em zeros; se a
      webcam parar de entregar gestos, manda zeros (não repete o último valor);
    - se a telemetria disser stick_mode=false (pânico, ASSUMIR CONTROLE, app em
      segundo plano) ou sumir, desarma também e NÃO rearma sozinho;
    - ao sair (q, Esc, Ctrl+C, SIGTERM, erro, webcam perdida) publica um Twist
      zerado e depois stick_mode=false.

Controle por teclado (janela da webcam em foco):
    a — arma/desarma o envio de comandos (equivalente ao antigo "stick mode")
    t — decolar: aperte duas vezes em até 3 s (também: polegar para cima por 2 s)
    l — pousar (também: polegar para baixo por 2 s)
    q / ESC — sai
"""

import json
import signal
import threading
import time
import uuid

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from .comando_voo import PARADO, ZEROS, FiltroRetorno, Suavizador, alvo_do_gesto
from .visao_mao import CONEXOES_MAO, DetectorGestos, Gesto

# Tempo que o polegar deve ficar para cima/baixo antes de decolar/pousar por gesto.
SEGURAR_DECOLAR_POUSAR_S = 2.0
# Segunda pressão de 't' deve vir neste prazo para confirmar a decolagem.
CONFIRMAR_DECOLAR_S = 3.0

PERIODO_CMD_S = 0.05  # 20 Hz
GESTO_VALIDO_S = 0.3  # gesto mais velho que isso (webcam travada) vira zeros
TELEMETRIA_VALIDA_S = 2.0  # o celular publica a 2 Hz
CARENCIA_ARMAR_S = 3.0  # o celular leva um instante para ligar o stick mode
MENSAGEM_S = 4.0  # quanto tempo um aviso fica na janela


class PilotoGestos(Node):
    """Nó ROS2: publica comandos de voo a partir do gesto estabilizado atual."""

    def __init__(self) -> None:
        super().__init__("piloto_gestos")
        self.declare_parameter("camera_index", 0)
        self._cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self._armado_pub = self.create_publisher(Bool, "/drone/stick_mode", 10)
        self._rpc_pub = self.create_publisher(String, "/drone/rpc/request", 10)
        self.create_subscription(String, "/phone/telemetria", self._ao_telemetria, 10)
        self.create_subscription(String, "/phone/rpc/response", self._ao_resposta_rpc, 10)

        self._trava = threading.RLock()  # o timer/callbacks rodam em outra thread (executor)
        self.armado = False
        self._armado_em = 0.0
        self._telemetria: dict = {}
        self._telemetria_em = -1e9
        self._gesto_nome = PARADO
        self._gesto_em = -1e9
        self._filtro = FiltroRetorno()
        self._suavizador = Suavizador()
        self._ultimo_passo = time.monotonic()
        self._inicio_polegar: float | None = None
        self._ultimo_gesto_polegar: Gesto | None = None
        self._decolar_pedido_em: float | None = None
        self._prefixo = uuid.uuid4().hex[:8]  # ids de RPC únicos por programa
        self._contador = 0
        self._pendentes: dict[str, str] = {}
        self._aviso = ""
        self._aviso_em = -1e9
        self.create_timer(PERIODO_CMD_S, self._passo)

    # ---- estado do celular -------------------------------------------------

    def _ao_telemetria(self, msg: String) -> None:
        try:
            dados = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(dados, dict):
            with self._trava:
                self._telemetria = dados
                self._telemetria_em = time.monotonic()

    def _celular_ouvindo(self, agora: float) -> bool:
        with self._trava:
            return agora - self._telemetria_em < TELEMETRIA_VALIDA_S

    def _avisar(self, texto: str, nivel: str = "info") -> None:
        with self._trava:
            self._aviso, self._aviso_em = texto, time.monotonic()
        log = self.get_logger()  # o rclpy não aceita severidades diferentes na mesma linha de código
        if nivel == "warning":
            log.warning(texto)
        elif nivel == "error":
            log.error(texto)
        else:
            log.info(texto)

    def linhas_status(self) -> list[str]:
        """Texto para a janela (ASCII: o OpenCV não desenha acentos)."""
        agora = time.monotonic()
        with self._trava:
            if not self._celular_ouvindo(agora):
                celular = "celular: SEM TELEMETRIA"
            else:
                t = self._telemetria
                partes = ["drone ok" if t.get("drone_conectado") else "DRONE DESCONECTADO"]
                if t.get("bateria_percent") is not None:
                    partes.append(f"bat {t['bateria_percent']}%")
                partes.append("voando" if t.get("voando") else "no chao")
                partes.append("stick ON" if t.get("stick_mode") else "stick off")
                celular = "celular: " + ", ".join(partes)
            linhas = [celular]
            if agora - self._aviso_em < MENSAGEM_S:
                linhas.append(self._aviso)
            return linhas

    # ---- armar / desarmar --------------------------------------------------

    def armar(self) -> bool:
        agora = time.monotonic()
        with self._trava:
            if self.armado:
                return True
            if not self._celular_ouvindo(agora):
                self._avisar("Nao armou: sem telemetria do celular.", "warning")
                return False
            if not self._telemetria.get("drone_conectado"):
                self._avisar("Nao armou: o celular nao enxerga o drone.", "warning")
                return False
            if self._armado_pub.get_subscription_count() == 0:
                self._avisar("Nao armou: o celular nao assinou /drone/stick_mode.", "warning")
                return False
            self._filtro.reiniciar()
            self._suavizador.reiniciar()
            self._gesto_nome, self._gesto_em = PARADO, agora
            self.armado, self._armado_em = True, agora
            self._armado_pub.publish(Bool(data=True))  # só na mudança de estado
            self._avisar("ARMADO")
            return True

    def desarmar(self, motivo: str = "DESARMADO") -> None:
        with self._trava:
            if not self.armado:
                return
            self._cmd_pub.publish(Twist())  # zera antes de soltar o controle
            self._armado_pub.publish(Bool(data=False))
            self.armado = False
            self._suavizador.reiniciar()
            self._avisar(motivo)

    def alternar_armado(self) -> None:
        if self.armado:
            self.desarmar()
        else:
            self.armar()

    # ---- gestos de movimento (thread da webcam) ----------------------------

    def definir_gesto(self, gesto: Gesto) -> None:
        agora = time.monotonic()
        with self._trava:
            self._gesto_nome = self._filtro.filtrar(gesto.name, agora)
            self._gesto_em = agora

    # ---- 20 Hz (thread do executor) ----------------------------------------

    def _passo(self) -> None:
        agora = time.monotonic()
        with self._trava:
            dt = min(agora - self._ultimo_passo, 0.2)
            self._ultimo_passo = agora
            if not self.armado:
                return
            if not self._celular_ouvindo(agora):
                self.desarmar("Desarmado: perdi a telemetria do celular.")
                return
            if agora - self._armado_em > CARENCIA_ARMAR_S and self._telemetria.get("stick_mode") is False:
                self.desarmar("Desarmado: o celular retomou o controle (stick mode desligado).")
                return
            alvo = ZEROS
            if agora - self._gesto_em <= GESTO_VALIDO_S:
                alvo = alvo_do_gesto(self._gesto_nome)
            x, y, z, yaw = self._suavizador.passo(alvo, dt)
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = x, y, z, yaw
            self._cmd_pub.publish(twist)  # 20 Hz mesmo em zeros: calar é como o app sabe que travou

    # ---- decolar / pousar por RPC (com resposta) ---------------------------

    def _pedir(self, cmd: str) -> None:
        with self._trava:
            self._contador += 1
            id_ = f"{self._prefixo}-{self._contador}"
            self._pendentes[id_] = cmd
        self._rpc_pub.publish(String(data=json.dumps({"id": id_, "cmd": cmd, "args": {}})))

    def _ao_resposta_rpc(self, msg: String) -> None:
        try:
            resposta = json.loads(msg.data)
        except ValueError:
            return
        with self._trava:
            cmd = self._pendentes.pop(resposta.get("id"), None)  # ignora respostas de outros programas
        if cmd is None:
            return
        if resposta.get("ok"):
            self._avisar(f"{cmd}: aceito pelo drone.")
        else:
            self._avisar(f"{cmd}: ERRO - {resposta.get('error')}", "error")

    def decolar(self, origem: str) -> None:
        agora = time.monotonic()
        with self._trava:
            if not self._celular_ouvindo(agora):
                self._avisar("Nao decolou: sem telemetria do celular.", "warning")
            elif not self._telemetria.get("drone_conectado"):
                self._avisar("Nao decolou: o celular nao enxerga o drone.", "warning")
            elif self._telemetria.get("voando"):
                self._avisar("Nao decolou: o drone ja esta no ar.", "warning")
            else:
                self._avisar(f"Decolar ({origem})")
                self._pedir("flight.take_off")

    def pousar(self, origem: str) -> None:
        if not self._celular_ouvindo(time.monotonic()):
            self._avisar("Nao pousou: sem telemetria do celular.", "warning")
            return
        self._avisar(f"Pousar ({origem})")
        self._pedir("flight.land")

    def tecla_decolar(self) -> None:
        """Duas pressões de 't' em até CONFIRMAR_DECOLAR_S."""
        agora = time.monotonic()
        if self._decolar_pedido_em is not None and agora - self._decolar_pedido_em <= CONFIRMAR_DECOLAR_S:
            self._decolar_pedido_em = None
            self.decolar("tecla")
        else:
            self._decolar_pedido_em = agora
            self._avisar("Aperte 't' de novo para confirmar a decolagem.")

    def avaliar_polegar(self, gesto: Gesto, agora: float) -> None:
        """Decola/pousa quando o polegar fica para cima/baixo por `SEGURAR_DECOLAR_POUSAR_S`."""
        if gesto not in (Gesto.POLEGAR_CIMA, Gesto.POLEGAR_BAIXO):
            self._inicio_polegar = None
            self._ultimo_gesto_polegar = None
            return
        if gesto != self._ultimo_gesto_polegar:
            self._ultimo_gesto_polegar = gesto
            self._inicio_polegar = agora
            return
        if self._inicio_polegar is not None and agora - self._inicio_polegar >= SEGURAR_DECOLAR_POUSAR_S:
            if gesto is Gesto.POLEGAR_CIMA:
                self.decolar("gesto")
            else:
                self.pousar("gesto")
            # Evita repetir o comando enquanto o polegar continuar no ar.
            self._inicio_polegar = None
            self._ultimo_gesto_polegar = None


def _encerrar(*_) -> None:
    """SIGINT/SIGTERM: sai pelo `finally` (que desarma) e ignora os sinais seguintes.

    Num Ctrl+C dentro do `ros2 launch` o nó recebe SIGINT duas vezes (o terminal manda ao grupo
    inteiro e o launch repassa); a segunda não pode interromper o desarme.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main() -> None:
    # Sem isso, no Humble o Ctrl+C invalida o contexto antes do `finally` e o desarme não é publicado.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _encerrar)
    signal.signal(signal.SIGTERM, _encerrar)
    no = PilotoGestos()
    executor = SingleThreadedExecutor()
    executor.add_node(no)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    captura = None
    try:
        captura = cv2.VideoCapture(int(no.get_parameter("camera_index").value))
        if not captura.isOpened():
            no.get_logger().error("Webcam indisponível.")
            return
        with DetectorGestos() as detector:
            while rclpy.ok():
                ok, frame = captura.read()
                if not ok:
                    no.get_logger().error("Webcam indisponível; encerrando.")
                    break
                frame = cv2.flip(frame, 1)
                gesto = detector.processar(frame)
                agora = time.monotonic()

                no.avaliar_polegar(gesto, agora)
                no.definir_gesto(gesto)

                if detector.pontos is not None:
                    pts = detector.pontos.astype(int)
                    for a, b in CONEXOES_MAO:
                        cv2.line(frame, tuple(pts[a]), tuple(pts[b]), (255, 255, 255), 2)
                estado = "ARMADO" if no.armado else "DESARMADO ('a' p/ armar)"
                cv2.putText(frame, f"{gesto.value} [{estado}]", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                for i, linha in enumerate(no.linhas_status()):
                    cv2.putText(frame, linha, (10, 60 + 28 * i),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
                cv2.imshow("Piloto por gestos", frame)

                tecla = cv2.waitKey(1) & 0xFF
                if tecla in (ord("q"), 27):
                    break
                elif tecla == ord("a"):
                    no.alternar_armado()
                elif tecla == ord("t"):
                    no.tecla_decolar()
                elif tecla == ord("l"):
                    no.pousar("tecla")
    except KeyboardInterrupt:
        pass
    finally:
        no.desarmar("Encerrando: desarmado.")  # Ctrl+C, SIGTERM, erro, webcam perdida ou saída normal
        time.sleep(0.2)  # dá tempo de o DDS entregar o Twist zerado e o stick_mode=false
        if captura is not None:
            captura.release()
        cv2.destroyAllWindows()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

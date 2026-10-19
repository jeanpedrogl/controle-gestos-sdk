"""Traduz os gestos de `visao_mao.py` em comandos reais de voo para o drone.

A imagem vem da **câmera do próprio drone**: o celular reenvia o vídeo em JPEG pelo tópico
`/phone/drone_camera/compressed`, este nó classifica o gesto da mão em cada quadro recebido e
publica os comandos em tópicos ROS2. A detecção é em duas etapas (pessoa -> recorte da mão, ver
`visao_mao.py`), porque na câmera grande-angular do drone a mão inteira ocupa poucos pixels. A ponte no celular (app Drone Bridge) assina esses
tópicos via rosbridge e repassa ao DJI SDK — nenhuma lógica de visão mora
aqui, isso fica inteiramente em `visao_mao.py`; a conversão gesto -> velocidade
(rampa) fica em `comando_voo.py`.

Convenções de eixo (REP 103, padrão ROS): x = frente, y = esquerda, z = cima,
yaw positivo = sentido anti-horário (giro para a esquerda). O lado do celular
é quem converte isso para as unidades/sinais que o SDK da DJI espera.

Segurança (regras do `MANUAL_DESENVOLVEDOR.md` do Drone Bridge):
    - começa desarmado; só arma com a tecla 'a' e só com o celular conectado
      (telemetria chegando e o celular assinando /drone/stick_mode);
    - enquanto armado, publica /drone/cmd_vel a 20 Hz, mesmo em zeros; se o vídeo
      do drone parar de chegar (ou ficar atrasado), manda zeros (não repete o último valor);
    - se a telemetria disser stick_mode=false (pânico, ASSUMIR CONTROLE, app em
      segundo plano) ou sumir, desarma também e NÃO rearma sozinho;
    - ao sair (q, Esc, Ctrl+C, SIGTERM, erro) publica um Twist
      zerado e depois stick_mode=false.

Controle por teclado (janela do vídeo do drone em foco):
    a — arma/desarma o envio de comandos (equivalente ao antigo "stick mode")
    t — decolar: aperte duas vezes em até 3 s (também: punho fechado mantido por 2 s, no chão)
    l — pousar (também: punho fechado mantido por 2 s, voando)
    1..4 — qualidade do vídeo do drone, da mais leve (menos atraso) à mais nítida (ver PRESETS_VIDEO)
    q / ESC — sai

Modo Fase 3 (parâmetro `modo:=fase3`; o padrão é `teclado`, descrito acima): quem decide é a máquina
de estados de `missao_fase3.py`. A cada 50 ms `_passo_fase3` monta uma `Entrada` (telemetria, gesto,
eventos de punho/dois punhos, a pessoa, confirmação de pouso), chama `MissaoFase3.passo` e executa a
`Saida`: arma/desarma, velocidade pela rampa, RPC (decolar, pousar, cancelar a decolagem), ângulo do
gimbal e as mensagens (coloridas no terminal, na janela e no log da tarefa em ~/.ros/log). Nenhuma
tecla funciona nesse modo, só q/ESC para sair (o regulamento proíbe tocar no computador durante a
tentativa); as regras de segurança acima continuam valendo, agora aplicadas pela máquina.
"""

import json
import os
import signal
import threading
import time
import uuid
from datetime import datetime

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from .comando_voo import PARADO, ZEROS, GestoMantido, Suavizador, alvo_do_gesto
from .missao_fase3 import Entrada, MissaoFase3, pessoa_do_corpo
from .terminal import colorir
from .visao_mao import DetectorGestos, Gesto, desenhar

# Tempo que o punho fechado deve ser mantido antes de decolar (no chão) ou pousar (voando) por gesto.
SEGURAR_PUNHO_S = 2.0
# Uma falha de detecção mais curta que isto no meio da mão fechada não zera a contagem (a regra
# acerta ~95% dos quadros, então sem tolerância um quadro errado em 2 s já cancelava tudo). Mudar de
# gesto por mais tempo que isso cancela, e para agir de novo é preciso soltar a mão por mais que isso.
TOLERANCIA_PUNHO_S = 0.4
# Depois de pedir a decolagem, não pede outra por este tempo (o `voando` da telemetria demora a virar true).
DECOLAGEM_PEDIDA_S = 10.0
# Segunda pressão de 't' deve vir neste prazo para confirmar a decolagem.
CONFIRMAR_DECOLAR_S = 3.0

PERIODO_CMD_S = 0.05  # 20 Hz
# Gesto mais velho que isso vira zeros. O vídeo do drone chega a ~8 fps (125 ms entre quadros) e
# o celular descarta quadros quando o Wi-Fi atrasa, então a margem é maior que com uma webcam.
GESTO_VALIDO_S = 0.6
VIDEO_PERDIDO_S = 1.0  # sem quadro novo há mais que isso: a janela avisa "SEM VIDEO"
FRAMES_CONFIRMACAO = 2  # quadros iguais seguidos para trocar de gesto (com ~8 fps, 3 seriam ~0,4 s)
TOPICO_VIDEO = "/phone/drone_camera/compressed"

# Teclas 1-4: qualidade do vídeo reenviado pelo celular (RPC `video.set_quality`). Mais fps e menos
# pixels = menos atraso, mas a mão fica pequena e borrada; mais pixels/qualidade = detecção melhor,
# mais atraso. Valores iniciais, a ajustar no campo (faixas aceitas: fps 1-30, largura 160-1920,
# qualidade JPEG 10-95). A 3 é o padrão do celular.
PRESETS_VIDEO = {
    1: ("minima", {"fps": 15, "max_width": 320, "jpeg_quality": 40}),
    2: ("baixa", {"fps": 12, "max_width": 480, "jpeg_quality": 50}),
    3: ("padrao", {"fps": 8, "max_width": 640, "jpeg_quality": 50}),
    4: ("alta", {"fps": 8, "max_width": 960, "jpeg_quality": 70}),
}
TELEMETRIA_VALIDA_S = 2.0  # o celular publica a 2 Hz
CARENCIA_ARMAR_S = 3.0  # o celular leva um instante para ligar o stick mode
MENSAGEM_S = 4.0  # quanto tempo um aviso fica na janela
# Estado do voo em /phone/state (RPC `state.stream_start`, seção "flight"): traz
# `landing_confirmation_needed`, que a telemetria de 2 Hz não tem. O celular para esse fluxo quando a
# ponte cai, então o nó pede de novo sempre que ele some.
INTERVALO_ESTADO_MS = 250
ESTADO_VALIDO_S = 1.0
REPEDIR_ESTADO_S = 2.0
# Modo Fase 3: o que cada `Saida.acao` da máquina pede ao celular, e o gimbal.
RPC_DA_ACAO = {"decolar": "flight.take_off", "pousar": "flight.land", "cancelar_decolagem": "flight.cancel_takeoff"}
GIMBAL_TEMPO_S = 1.0  # duração do movimento do gimbal até o ângulo pedido
REPETIR_GIMBAL_S = 2.0  # depois de um erro do gimbal, espera isso antes de pedir de novo


class PilotoGestos(Node):
    """Nó ROS2: publica comandos de voo a partir do gesto estabilizado atual."""

    def __init__(self) -> None:
        super().__init__("piloto_gestos")
        self._cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self._armado_pub = self.create_publisher(Bool, "/drone/stick_mode", 10)
        self._rpc_pub = self.create_publisher(String, "/drone/rpc/request", 10)
        self.create_subscription(String, "/phone/telemetria", self._ao_telemetria, 10)
        self.create_subscription(String, "/phone/rpc/response", self._ao_resposta_rpc, 10)
        self.create_subscription(String, "/phone/state", self._ao_estado, 10)
        # Fila de 1 e best-effort: só interessa o quadro mais recente (compatível com qualquer QoS do rosbridge).
        qos_video = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, TOPICO_VIDEO, self._ao_video, qos_video)

        self._trava = threading.RLock()  # o timer/callbacks rodam em outra thread (executor)
        self.armado = False
        self._armado_em = 0.0
        self._telemetria: dict = {}
        self._telemetria_em = -1e9
        self._gesto_nome = PARADO
        self._gesto_em = -1e9
        self._jpeg: bytes | None = None  # quadro mais recente do drone, ainda não decodificado
        self._jpeg_em = -1e9
        self._jpeg_seq = 0
        self._suavizador = Suavizador()
        self._ultimo_passo = time.monotonic()
        self._inicio_punho: float | None = None
        self._punho_visto_em = -1e9  # último quadro avaliado com o punho fechado
        self._punho_disparado = False  # só age de novo depois de soltar o punho
        self._punho_avisado = False  # o motivo de um punho ignorado é avisado uma vez por punho
        self._decolagem_pedida_em = -1e9
        self._video_texto = "video: padrao do celular (teclas 1-4 mudam)"
        self._decolar_pedido_em: float | None = None
        self._prefixo = uuid.uuid4().hex[:8]  # ids de RPC únicos por programa
        self._contador = 0
        self._pendentes: dict[str, tuple[str, int | None]] = {}  # id -> (comando, preset de vídeo)
        self._aviso = ""
        self._aviso_em = -1e9
        self._confirmar_pouso = False  # o drone parou perto do chão esperando confirmação de pouso
        self._estado_em = -1e9  # última mensagem de /phone/state
        self._estado_pedido_em = -1e9

        modo = self.declare_parameter("modo", "teclado").value
        if modo not in ("teclado", "fase3"):
            raise ValueError(f"modo deve ser 'teclado' ou 'fase3', nao {modo!r}")
        self.modo_fase3 = modo == "fase3"
        self.missao = MissaoFase3()
        self._contador_punho = GestoMantido("PUNHO_FECHADO", SEGURAR_PUNHO_S, TOLERANCIA_PUNHO_S, GESTO_VALIDO_S)
        self._contador_voltar = GestoMantido("DOIS_PUNHOS", SEGURAR_PUNHO_S, TOLERANCIA_PUNHO_S, GESTO_VALIDO_S)
        self._evento_punho = False  # eventos da thread principal, entregues à máquina no próximo passo
        self._evento_voltar = False
        self._pessoa = None  # Pessoa do último quadro com exatamente uma pessoa
        self._pessoa_em = -1e9
        self._gimbal_enviado: float | None = None
        self._extensao_gimbal_pedida = False
        self._gimbal_tentar_em = 0.0
        self._log_tarefa = self._abrir_log_tarefa() if self.modo_fase3 else None

        self.create_timer(PERIODO_CMD_S, self._passo)
        self.create_timer(1.0, self._manter_estado)

    # ---- vídeo do drone ----------------------------------------------------

    def _ao_video(self, msg: CompressedImage) -> None:
        """Só guarda o JPEG (rápido): a decodificação e a detecção rodam na thread principal."""
        with self._trava:
            self._jpeg, self._jpeg_em = bytes(msg.data), time.monotonic()
            self._jpeg_seq += 1

    def quadro_novo(self, ultima_seq: int) -> tuple[int, np.ndarray, float] | None:
        """(seq, imagem BGR, instante em que chegou) se chegou um quadro depois de `ultima_seq`.

        Pula os quadros intermediários: se a detecção é mais lenta que o vídeo, vale o mais recente.
        """
        with self._trava:
            if self._jpeg is None or self._jpeg_seq == ultima_seq:
                return None
            jpeg, seq, instante = self._jpeg, self._jpeg_seq, self._jpeg_em
        imagem = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if imagem is None:  # JPEG corrompido: ignora e espera o próximo
            return None
        return seq, imagem, instante

    def idade_video(self, agora: float) -> float | None:
        """Segundos desde o último quadro recebido, ou None se nunca chegou nenhum."""
        with self._trava:
            return None if self._jpeg is None else agora - self._jpeg_em

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

    def _ao_estado(self, msg: String) -> None:
        """/phone/state: só interessa `flight.landing_confirmation_needed` (proteção de pouso da DJI)."""
        try:
            dados = json.loads(msg.data)
        except ValueError:
            return
        voo = dados.get("flight") if isinstance(dados, dict) else None
        pede = isinstance(voo, dict) and voo.get("landing_confirmation_needed") is True
        with self._trava:
            novo_pedido = pede and not self._confirmar_pouso
            self._confirmar_pouso, self._estado_em = pede, time.monotonic()
        if novo_pedido:  # só no terminal: a janela já tem uma linha fixa enquanto o pedido durar
            self.get_logger().warning("DRONE PEDE CONFIRMACAO DE POUSO: feche o punho (ou 'l')")

    def _manter_estado(self) -> None:
        """1 Hz: pede /phone/state de novo se ele sumiu (o celular para o fluxo quando a ponte cai)."""
        agora = time.monotonic()
        with self._trava:
            if not self._celular_ouvindo(agora) or agora - self._estado_em <= ESTADO_VALIDO_S:
                return
            if agora - self._estado_pedido_em < REPEDIR_ESTADO_S:
                return
            self._estado_pedido_em = agora
        self._pedir("state.stream_start", {"interval_ms": INTERVALO_ESTADO_MS, "sections": ["flight"]})

    def confirmacao_pouso_pendente(self, agora: float) -> bool:
        """O drone espera confirmação de pouso (falso se /phone/state está velho). Para a `Entrada`."""
        with self._trava:
            return self._confirmar_pouso and agora - self._estado_em <= ESTADO_VALIDO_S

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
            linhas = [celular, self._video_texto]
            if self.confirmacao_pouso_pendente(agora):
                linhas.append("DRONE PEDE CONFIRMACAO DE POUSO: punho fechado ou 'l'")
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

    # ---- gesto atual (thread principal, a partir dos quadros do drone) ----------------------------

    def definir_gesto(self, gesto: Gesto, instante: float) -> None:
        """`instante`: quando o quadro chegou (não quando a detecção terminou), para a idade contar a latência."""
        with self._trava:
            self._gesto_nome = gesto.name
            self._gesto_em = instante

    # ---- 20 Hz (thread do executor) ----------------------------------------

    def _passo(self) -> None:
        agora = time.monotonic()
        with self._trava:
            dt = min(agora - self._ultimo_passo, 0.2)
            self._ultimo_passo = agora
            if self.modo_fase3:
                self._passo_fase3(agora, dt)
                return
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

    # ---- modo Fase 3: a máquina de estados decide ----------------------------

    def _passo_fase3(self, agora: float, dt: float) -> None:
        """20 Hz: Entrada -> MissaoFase3.passo -> executa a Saida. Chamado com a trava tomada."""
        t = self._telemetria
        ouvindo = self._celular_ouvindo(agora)
        entrada = Entrada(
            agora=agora,
            telemetria_ok=ouvindo,
            drone_conectado=bool(t.get("drone_conectado")),
            voando=t.get("voando"),
            stick_mode=t.get("stick_mode"),
            stick_assinado=self._armado_pub.get_subscription_count() > 0,
            gesto=self._gesto_nome if agora - self._gesto_em <= GESTO_VALIDO_S else PARADO,
            punho=self._evento_punho,
            voltar=self._evento_voltar,
            pessoa=self._pessoa if agora - self._pessoa_em <= GESTO_VALIDO_S else None,
            confirmar_pouso=self.confirmacao_pouso_pendente(agora),
        )
        self._evento_punho = self._evento_voltar = False  # cada evento vale para um passo só
        saida = self.missao.passo(entrada)

        for msg in saida.mensagens:
            self._mostrar(msg)
        if saida.armado and not self.armado:  # armado/gimbal: estados desejados, publicados na mudança
            self.armado, self._armado_em = True, agora
            self._suavizador.reiniciar()
            self._armado_pub.publish(Bool(data=True))
        elif not saida.armado and self.armado:
            self.desarmar("Desarmado pela missao.")  # Twist zerado e depois stick_mode=false
        if self.armado:  # 20 Hz mesmo em zeros: calar é como o app sabe que travou
            x, y, z, yaw = self._suavizador.passo(saida.velocidade, dt)
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = x, y, z, yaw
            self._cmd_pub.publish(twist)
        if saida.acao is not None:  # evento: vem num passo só
            self._pedir(RPC_DA_ACAO[saida.acao])
        if ouvindo and t.get("drone_conectado"):
            self._ajustar_gimbal(saida.gimbal_pitch, agora)

    def _ajustar_gimbal(self, pitch: float, agora: float) -> None:
        """Liga a extensão (acima de 0°) uma vez e manda o ângulo só quando ele muda."""
        if agora < self._gimbal_tentar_em:
            return
        if not self._extensao_gimbal_pedida:
            self._extensao_gimbal_pedida = True
            self._pedir("gimbal.set_pitch_range_extension", {"enabled": True})
        if pitch != self._gimbal_enviado:
            self._gimbal_enviado = pitch
            self._pedir("gimbal.rotate", {"mode": "ABSOLUTE_ANGLE", "pitch": pitch, "time": GIMBAL_TEMPO_S})

    def _mostrar(self, msg: str) -> None:
        """Uma mensagem da missão: colorida no terminal, crua na janela e no log da tarefa."""
        print(colorir(msg), flush=True)
        self._aviso, self._aviso_em = msg, time.monotonic()
        if self._log_tarefa is not None:
            self._log_tarefa.write(f"{datetime.now():%H:%M:%S}  {msg}\n")

    def _abrir_log_tarefa(self):
        """Arquivo de texto com as mensagens da tentativa (o regulamento pede o log das atividades)."""
        pasta = os.path.expanduser("~/.ros/log")
        os.makedirs(pasta, exist_ok=True)
        caminho = os.path.join(pasta, f"fase3-{datetime.now():%Y%m%d-%H%M%S}.txt")
        arquivo = open(caminho, "a", encoding="utf-8", buffering=1)  # linha a linha: sobrevive a um crash
        self.get_logger().info(f"Log da tarefa: {caminho}")
        return arquivo

    def avaliar_fase3(self, gesto: Gesto, corpos: list[np.ndarray], forma: tuple[int, int], instante: float) -> None:
        """Thread principal, a cada quadro novo: eventos de punho/dois punhos e a pessoa para a máquina."""
        punho = self._contador_punho.avaliar(gesto.name, instante)
        voltar = self._contador_voltar.avaliar(gesto.name, instante)
        pessoa = None
        if len(corpos) == 1:  # zero ou duas pessoas: nada (fica velho e vira None na Entrada)
            altura, largura = forma
            pessoa = pessoa_do_corpo(corpos[0], largura, altura)
        with self._trava:
            self._evento_punho |= punho  # |=: um evento não se perde se o passo ainda não o leu
            self._evento_voltar |= voltar
            if pessoa is not None:
                self._pessoa, self._pessoa_em = pessoa, instante

    def progresso_fase3(self, agora: float) -> tuple[str, float] | None:
        """(nome, fração) do punho ou dos dois punhos em contagem, para a janela."""
        for nome, contador in (("Punho", self._contador_punho), ("Dois punhos", self._contador_voltar)):
            fracao = contador.progresso(agora)
            if fracao is not None:
                return nome, fracao
        return None

    def destroy_node(self) -> None:
        if self._log_tarefa is not None:
            self._log_tarefa.close()
        super().destroy_node()

    # ---- decolar / pousar por RPC (com resposta) ---------------------------

    def _pedir(self, cmd: str, args: dict | None = None, preset: int | None = None) -> None:
        with self._trava:
            self._contador += 1
            id_ = f"{self._prefixo}-{self._contador}"
            self._pendentes[id_] = (cmd, preset)
        self._rpc_pub.publish(String(data=json.dumps({"id": id_, "cmd": cmd, "args": args or {}})))

    def _ao_resposta_rpc(self, msg: String) -> None:
        try:
            resposta = json.loads(msg.data)
        except ValueError:
            return
        with self._trava:
            pedido = self._pendentes.pop(resposta.get("id"), None)  # ignora respostas de outros programas
        if pedido is None:
            return
        cmd, preset = pedido
        if not resposta.get("ok"):
            if cmd == "flight.take_off":
                with self._trava:
                    self._decolagem_pedida_em = -1e9  # recusada: libera uma nova tentativa
            elif cmd.startswith("gimbal."):
                with self._trava:  # `_ajustar_gimbal` pede de novo daqui a pouco
                    self._gimbal_enviado, self._gimbal_tentar_em = None, time.monotonic() + REPETIR_GIMBAL_S
                    if cmd == "gimbal.set_pitch_range_extension":
                        self._extensao_gimbal_pedida = False
            self._avisar(f"{cmd}: ERRO - {resposta.get('error')}", "error")
        elif cmd == "state.stream_start" or cmd.startswith("gimbal."):
            pass  # pedidos automáticos (`_manter_estado`, `_ajustar_gimbal`): só os erros aparecem
        elif cmd == "video.set_quality" and isinstance(resposta.get("result"), dict):
            r = resposta["result"]  # valores em vigor no celular
            nome = PRESETS_VIDEO[preset][0] if preset in PRESETS_VIDEO else "?"
            texto = f"video {preset} ({nome}): {r.get('fps')} fps, {r.get('max_width')} px, q{r.get('jpeg_quality')}"
            with self._trava:
                self._video_texto = texto
            self.get_logger().info(texto)  # a janela já mostra `_video_texto` em linha própria
        else:
            self._avisar(f"{cmd}: aceito pelo drone.")

    def definir_qualidade_video(self, preset: int) -> None:
        """Tecla 1-4: pede ao celular o preset de `PRESETS_VIDEO`; a janela mostra o que ele confirmou."""
        if preset not in PRESETS_VIDEO:
            return
        if not self._celular_ouvindo(time.monotonic()):
            self._avisar("Video nao mudou: sem telemetria do celular.", "warning")
            return
        nome, args = PRESETS_VIDEO[preset]
        self._avisar(f"Pedindo video {preset} ({nome})...")
        self._pedir("video.set_quality", args, preset)

    def decolar(self, origem: str) -> None:
        agora = time.monotonic()
        with self._trava:
            if not self._celular_ouvindo(agora):
                self._avisar("Nao decolou: sem telemetria do celular.", "warning")
            elif not self._telemetria.get("drone_conectado"):
                self._avisar("Nao decolou: o celular nao enxerga o drone.", "warning")
            elif self._telemetria.get("voando"):
                self._avisar("Nao decolou: o drone ja esta no ar.", "warning")
            elif agora - self._decolagem_pedida_em < DECOLAGEM_PEDIDA_S:
                self._avisar("Nao decolou: a decolagem ja foi pedida.", "warning")
            else:
                self._decolagem_pedida_em = agora
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

    def acao_punho(self) -> tuple[str | None, str]:
        """(ação, motivo) do punho mantido agora.

        Ação: "Decolando" (no chão, e só com o programa armado), "Pousando" (voando; vale armado ou
        não, pousar nunca fica bloqueado) ou None, caso em que o motivo diz por quê. Decolar por gesto
        exige 'a' antes: um punho solto na imagem do drone não pode tirar o drone do chão sozinho.
        """
        with self._trava:
            if not self._celular_ouvindo(time.monotonic()):
                return None, "Punho ignorado: sem telemetria do celular."
            if self._telemetria.get("voando"):
                return "Pousando", ""
            if not self.armado:
                return None, "Punho ignorado: arme com 'a' para decolar por gesto (ou use 't' duas vezes)."
            return "Decolando", ""

    def avaliar_punho(self, gesto: Gesto, agora: float) -> None:
        """Decola (no chão, armado) ou pousa (voando) quando o punho fica fechado por `SEGURAR_PUNHO_S`.

        Age uma vez por punho: para repetir é preciso soltá-lo antes. Falhas de até
        `TOLERANCIA_PUNHO_S` no meio não contam como soltar. `agora` é o instante de chegada do quadro;
        só quadros novos chamam isto.
        """
        if gesto is not Gesto.PUNHO_FECHADO:
            if agora - self._punho_visto_em > TOLERANCIA_PUNHO_S:  # soltou de verdade, não foi um quadro ruim
                self._inicio_punho = None
                self._punho_disparado = False
                self._punho_avisado = False
            return
        if self._punho_disparado:
            self._punho_visto_em = agora  # continua fechada: segue valendo como o mesmo punho
            return
        acao, motivo = self.acao_punho()
        if acao is None:  # não conta tempo: só avisa (uma vez por punho) o que falta
            self._inicio_punho = None
            self._punho_visto_em = agora
            if not self._punho_avisado:
                self._punho_avisado = True
                self._avisar(motivo, "warning")
            return
        if self._inicio_punho is not None and agora - self._punho_visto_em > GESTO_VALIDO_S:
            self._inicio_punho = None  # o vídeo travou no meio: a contagem recomeça, não "pula" o tempo parado
        self._punho_visto_em = agora
        if self._inicio_punho is None:
            self._inicio_punho = agora
        elif agora - self._inicio_punho >= SEGURAR_PUNHO_S:
            self._punho_disparado = True
            if acao == "Decolando":
                self.decolar("gesto")
            else:
                self.pousar("gesto")

    def progresso_punho(self, agora: float) -> float | None:
        """Fração [0, 1) do tempo de punho já cumprido, ou None se não há contagem em andamento."""
        if self._inicio_punho is None or self._punho_disparado:
            return None
        return min((agora - self._inicio_punho) / SEGURAR_PUNHO_S, 0.99)


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
    try:
        with DetectorGestos(frames_confirmacao=FRAMES_CONFIRMACAO) as detector:
            seq = 0
            gesto = Gesto.NENHUMA_MAO
            quadro = np.zeros((360, 640, 3), dtype=np.uint8)  # até o primeiro quadro do drone chegar
            while rclpy.ok():
                novo = no.quadro_novo(seq)
                if novo is not None:
                    seq, imagem, instante = novo
                    quadro = cv2.flip(imagem, 1)  # espelha: esquerda/direita do ponto de vista de quem aparece
                    gesto = detector.processar(quadro, instante)
                    if no.modo_fase3:
                        no.avaliar_fase3(gesto, detector.corpos, quadro.shape[:2], instante)
                    else:
                        no.avaliar_punho(gesto, instante)
                    no.definir_gesto(gesto, instante)
                    desenhar(quadro, detector)  # corpo, recortes das mãos e mãos

                agora = time.monotonic()
                tela = quadro.copy()  # o HUD é redesenhado a cada volta, com ou sem quadro novo
                idade = no.idade_video(agora)
                if idade is None:
                    cv2.putText(tela, f"Aguardando video do drone em {TOPICO_VIDEO}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                elif idade > VIDEO_PERDIDO_S:
                    cv2.putText(tela, f"SEM VIDEO ha {idade:.0f} s - drone pairando", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    if no.modo_fase3:
                        estado = f"FASE 3: {no.missao.estado.name}, {no.missao.bases_visitadas} bases"
                    else:
                        estado = "ARMADO" if no.armado else "DESARMADO ('a' p/ armar)"
                    cv2.putText(tela, f"{gesto.value} [{estado}] ({detector.modo})", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                if no.modo_fase3:
                    contagem = no.progresso_fase3(agora)
                    acao, progresso = contagem if contagem else (None, None)
                else:
                    progresso = no.progresso_punho(agora)
                    acao = no.acao_punho()[0] or "Punho" if progresso is not None else None
                if progresso is not None and idade is not None and idade <= VIDEO_PERDIDO_S:
                    cv2.putText(tela, f"{acao}... {int(progresso * 100)}% (mude o gesto p/ cancelar)", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                for i, linha in enumerate(no.linhas_status()):
                    cv2.putText(tela, linha, (10, 90 + 28 * i),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
                rodape = ("FASE 3: teclado desligado (q/Esc sai e desarma)" if no.modo_fase3
                          else "Video: " + " | ".join(f"{k} {v[0]}" for k, v in PRESETS_VIDEO.items()))
                cv2.putText(tela, rodape, (10, tela.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 255), 1)
                cv2.imshow("Piloto por gestos (camera do drone)", tela)

                tecla = cv2.waitKey(1 if novo is not None else 10) & 0xFF
                if tecla in (ord("q"), 27):
                    break
                elif no.modo_fase3:
                    pass  # Fase 3: nenhuma outra tecla (não se toca no computador durante a tentativa)
                elif tecla == ord("a"):
                    no.alternar_armado()
                elif tecla == ord("t"):
                    no.tecla_decolar()
                elif tecla == ord("l"):
                    no.pousar("tecla")
                elif tecla in (ord(str(k)) for k in PRESETS_VIDEO):
                    no.definir_qualidade_video(tecla - ord("0"))
    except KeyboardInterrupt:
        pass
    finally:
        no.desarmar("Encerrando: desarmado.")  # Ctrl+C, SIGTERM, erro ou saída normal
        time.sleep(0.2)  # dá tempo de o DDS entregar o Twist zerado e o stick_mode=false
        cv2.destroyAllWindows()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

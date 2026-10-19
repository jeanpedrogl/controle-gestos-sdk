"""Lógica pura (sem ROS, sem webcam, sem MediaPipe) que transforma gestos em velocidades.

Fica separada de `integracao_visao_drone.py` para poder ser testada em milissegundos
(`test/test_comando_voo.py`). Os gestos entram pelo *nome* do `Gesto` (`gesto.name`), assim
este módulo não importa o MediaPipe.

Convenção de eixos (REP 103): x = frente, y = esquerda, z = cima, yaw+ = anti-horário.
Todos os valores são normalizados em [-1, 1]; o celular multiplica pelos limites reais.
"""

import math

# Velocidades normalizadas [0, 1] (1,0 = 4 m/s na horizontal, 2 m/s na vertical, 85 °/s no giro).
VELOCIDADE_HORIZONTAL = 0.2
VELOCIDADE_VERTICAL = 0.2
VELOCIDADE_YAW = 0.2
# O hang loose leva o drone para a frente, na direção de quem faz o gesto (a câmera que vê a mão é a
# do drone): metade da velocidade horizontal. Para trás continua a velocidade cheia.
FATOR_FRENTE = 0.5

# Rampa: tempo para ir de 0 até a velocidade máxima e de volta a 0.
TEMPO_SUBIDA_S = 0.3
TEMPO_DESCIDA_S = 0.15

PARADO = "NENHUMA_MAO"  # gesto "sem comando": zeros

# nome do Gesto -> (x, y, z, yaw) em múltiplos da velocidade de cada eixo
_DIRECAO = {
    "INDICADOR_CIMA": (0, 0, 1, 0),
    "INDICADOR_BAIXO": (0, 0, -1, 0),
    # O drone está de frente para quem faz o gesto (é a câmera dele que vê a mão), então a esquerda
    # da pessoa é a direita do drone: polegar para a direita (da pessoa) = y+ (esquerda do drone).
    "POLEGAR_ESQUERDA": (0, -1, 0, 0),
    "POLEGAR_DIREITA": (0, 1, 0, 0),
    "HANG_LOOSE": (FATOR_FRENTE, 0, 0, 0),  # para frente, devagar
    "MAO_ABERTA": (-1, 0, 0, 0),  # palma para a câmera: para trás
    "V": (0, 0, 0, 1),  # sempre anti-horário: a câmera gira com o drone, então o lado da mão na imagem não serve
}

# O que cada gesto de movimento faz, para o terminal (ASCII: também pode ir para a janela do OpenCV).
# Tem que cobrir exatamente as chaves de _DIRECAO (`test_comando_voo.py` confere).
DESCRICAO = {
    "INDICADOR_CIMA": "subir",
    "INDICADOR_BAIXO": "descer",
    "POLEGAR_ESQUERDA": "ir para a direita do drone (sua esquerda)",
    "POLEGAR_DIREITA": "ir para a esquerda do drone (sua direita)",
    "HANG_LOOSE": "ir para frente, devagar",
    "MAO_ABERTA": "ir para tras",
    "V": "girar no sentido anti-horario",
}

ZEROS = (0.0, 0.0, 0.0, 0.0)


def limitar(valor: float) -> float:
    """Corta em [-1, 1] e troca NaN/infinito por 0."""
    if not math.isfinite(valor):
        return 0.0
    return max(-1.0, min(1.0, valor))


def alvo_do_gesto(nome: str) -> tuple[float, float, float, float]:
    """(x, y, z, yaw) pedido pelo gesto; gestos de parada ou desconhecidos dão zeros."""
    direcao = _DIRECAO.get(nome)
    if direcao is None:
        return ZEROS
    dx, dy, dz, dyaw = direcao
    return (
        limitar(dx * VELOCIDADE_HORIZONTAL),
        limitar(dy * VELOCIDADE_HORIZONTAL),
        limitar(dz * VELOCIDADE_VERTICAL),
        limitar(dyaw * VELOCIDADE_YAW),
    )


class Suavizador:
    """Limita a variação de cada eixo por segundo (rampa de subida e de descida)."""

    def __init__(
        self,
        maximo: float = max(VELOCIDADE_HORIZONTAL, VELOCIDADE_VERTICAL, VELOCIDADE_YAW),
        tempo_subida_s: float = TEMPO_SUBIDA_S,
        tempo_descida_s: float = TEMPO_DESCIDA_S,
    ) -> None:
        self._taxa_subida = maximo / tempo_subida_s
        self._taxa_descida = maximo / tempo_descida_s
        self.atual = ZEROS

    def reiniciar(self) -> None:
        self.atual = ZEROS

    def passo(self, alvo: tuple[float, ...], dt: float) -> tuple[float, float, float, float]:
        saida = []
        for atual, desejado in zip(self.atual, alvo):
            desejado = limitar(desejado)
            if atual * desejado < 0:
                desejado = 0.0  # inverter o sentido passa primeiro por zero
            taxa = self._taxa_descida if abs(desejado) < abs(atual) else self._taxa_subida
            delta = max(-taxa * dt, min(taxa * dt, desejado - atual))
            saida.append(limitar(atual + delta))
        self.atual = tuple(saida)
        return self.atual


class GestoMantido:
    """Evento único quando o gesto `alvo` fica mantido `segurar_s` (punho, dois punhos).

    Mesmas regras do punho do modo teclado (`avaliar_punho` no nó): falhas de detecção de até
    `tolerancia_s` no meio não zeram a contagem nem contam como soltar; um intervalo maior que
    `quadro_velho_s` entre quadros (vídeo travou) recomeça a contagem; depois de disparar, só dispara
    de novo quando o gesto for solto. `agora` é o instante de chegada do quadro.
    """

    def __init__(self, alvo: str, segurar_s: float, tolerancia_s: float, quadro_velho_s: float) -> None:
        self.alvo = alvo
        self.segurar_s = segurar_s
        self.tolerancia_s = tolerancia_s
        self.quadro_velho_s = quadro_velho_s
        self._inicio: float | None = None
        self._visto_em = -1e9
        self._disparado = False

    def avaliar(self, nome: str, agora: float) -> bool:
        """True só no quadro em que o gesto completa `segurar_s`."""
        if nome != self.alvo:
            if agora - self._visto_em > self.tolerancia_s:  # soltou de verdade, não foi um quadro ruim
                self._inicio = None
                self._disparado = False
            return False
        if self._disparado:
            self._visto_em = agora
            return False
        if self._inicio is not None and agora - self._visto_em > self.quadro_velho_s:
            self._inicio = None  # o vídeo travou no meio: recomeça, não "pula" o tempo parado
        self._visto_em = agora
        if self._inicio is None:
            self._inicio = agora
            return False
        if agora - self._inicio >= self.segurar_s:
            self._disparado = True
            return True
        return False

    def progresso(self, agora: float) -> float | None:
        """Fração [0, 1) já cumprida, ou None sem contagem em andamento (para a janela)."""
        if self._inicio is None or self._disparado or agora - self._visto_em > self.tolerancia_s:
            return None
        return min((agora - self._inicio) / self.segurar_s, 0.99)

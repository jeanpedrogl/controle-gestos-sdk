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

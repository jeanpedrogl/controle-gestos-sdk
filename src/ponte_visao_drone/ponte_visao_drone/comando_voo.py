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

# Rampa: tempo para ir de 0 até a velocidade máxima e de volta a 0.
TEMPO_SUBIDA_S = 0.3
TEMPO_DESCIDA_S = 0.15

# Depois de um movimento, o gesto oposto dentro deste prazo é o "retorno" da mão ao centro,
# não um comando (mão sobe -> volta descendo -> o drone não deve descer).
RETORNO_S = 0.8

PARADO = "MAO_ABERTA_PARADA"

# nome do Gesto -> (x, y, z, yaw) em múltiplos da velocidade de cada eixo
_DIRECAO = {
    "MAO_ESQUERDA": (0, 1, 0, 0),
    "MAO_DIREITA": (0, -1, 0, 0),
    "MAO_SUBINDO": (0, 0, 1, 0),
    "MAO_DESCENDO": (0, 0, -1, 0),
    "MAO_APROXIMANDO": (1, 0, 0, 0),  # palma cresceu: drone vai para frente
    "MAO_AFASTANDO": (-1, 0, 0, 0),  # palma encolheu: drone vai para trás
    "V_ESQUERDA": (0, 0, 0, 1),
    "V_DIREITA": (0, 0, 0, -1),
}

# Pares de gestos de movimento opostos (a mão volta ao centro fazendo o oposto).
_OPOSTOS = {
    "MAO_ESQUERDA": "MAO_DIREITA",
    "MAO_DIREITA": "MAO_ESQUERDA",
    "MAO_SUBINDO": "MAO_DESCENDO",
    "MAO_DESCENDO": "MAO_SUBINDO",
    "MAO_APROXIMANDO": "MAO_AFASTANDO",
    "MAO_AFASTANDO": "MAO_APROXIMANDO",
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


class FiltroRetorno:
    """Ignora o gesto oposto que acontece logo depois de um movimento (a mão voltando)."""

    def __init__(self, janela_s: float = RETORNO_S) -> None:
        self._janela_s = janela_s
        self._ultimo: str | None = None
        self._instante = 0.0

    def reiniciar(self) -> None:
        self._ultimo = None

    def filtrar(self, nome: str, agora: float) -> str:
        if nome in _OPOSTOS:
            if _OPOSTOS[nome] == self._ultimo and agora - self._instante < self._janela_s:
                return PARADO
            self._ultimo, self._instante = nome, agora
        return nome


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

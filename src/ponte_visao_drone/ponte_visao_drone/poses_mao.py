"""Gestos estáticos da mão a partir dos 21 landmarks do MediaPipe (só numpy, sem MediaPipe).

Fica separado de `visao_mao.py` para poder ser testado sem modelo nem webcam
(`test/test_poses_mao.py`). Cada gesto é uma *pose* — nada depende de movimento —, então o
drone só anda enquanto a pose é mantida:

    indicador esticado, demais fechados, apontando p/ cima ou p/ baixo -> INDICADOR_CIMA / _BAIXO
    punho fechado com o polegar esticado na horizontal                 -> POLEGAR_ESQUERDA / _DIREITA
    "V" (indicador + médio)                                            -> V
    polegar e mínimo esticados, demais fechados ("hang loose")         -> HANG_LOOSE
    mão aberta com os dedos para cima                                  -> MAO_ABERTA
    punho fechado com o polegar recolhido                              -> PUNHO_FECHADO

Esquerda/direita (polegar) são as da imagem *espelhada* (o frame da webcam é invertido antes de chegar
aqui), ou seja, as do ponto de vista do usuário. Qualquer pose que não bata exatamente com uma
dessas é DESCONHECIDO (o drone paira). Mais de uma mão na imagem (por exemplo outra pessoa
entrando no quadro da câmera do drone) não é comando nenhum: VARIAS_MAOS, o drone paira e a
contagem do punho cai. Com mais de uma pessoa no quadro (`visao_mao.py`, via MediaPipe Pose)
vale o mesmo: VARIAS_PESSOAS.
"""

from enum import Enum

import numpy as np

FATOR_ESTICADO = 1.15  # dist(ponta, pulso) / dist(PIP, pulso) acima disso = dedo esticado
FATOR_DOBRADO = 1.0  # abaixo disso = dedo dobrado
FATOR_POLEGAR_ESTICADO = 0.55  # dist(ponta do polegar, base do indicador) / palma acima disso = polegar esticado
# Mão fechada = os 4 dedos dobrados e o polegar recolhido, que exige as duas coisas:
#   - a ponta do polegar não sai (mais que POLEGAR_FORA_MAX palmas) da caixa formada pelos outros
#     pontos da mão: polegar para cima, para baixo ou para o lado sai dela;
#   - a ponta do polegar está a menos de FATOR_POLEGAR_RECOLHIDO palmas do centro da palma.
# Calibrado com uma gravação de mãos fechadas (polegar por cima dos dedos, ao lado do indicador,
# pulso girado) e de gestos de polegar: o limite antigo (0,5 do centro, sem a caixa) recusava ~1/3
# das mãos fechadas, porque o polegar por cima dos dedos fica a 0,4-0,8 palma do centro.
POLEGAR_FORA_MAX = 0.15
FATOR_POLEGAR_RECOLHIDO = 0.85
LIMIAR_HORIZONTAL = 0.8  # |cos| do ângulo com a horizontal: polegar a até ~37° dela conta como lateral
LIMIAR_VERTICAL = 0.8  # idem para o indicador / a mão apontando para cima ou para baixo

PULSO, POLEGAR_MCP, POLEGAR_PONTA = 0, 2, 4
INDICADOR_MCP, INDICADOR_PONTA, MEDIO_MCP = 5, 8, 9
PALMA = [0, 5, 9, 13, 17]  # pulso + base de cada dedo
DEDOS = {"indicador": (6, 8), "medio": (10, 12), "anelar": (14, 16), "minimo": (18, 20)}  # (PIP, ponta)


class Gesto(Enum):
    NENHUMA_MAO = "Nenhuma mao"
    PUNHO_FECHADO = "Punho fechado"
    INDICADOR_CIMA = "Indicador para cima"
    INDICADOR_BAIXO = "Indicador para baixo"
    POLEGAR_ESQUERDA = "Polegar para a esquerda"
    POLEGAR_DIREITA = "Polegar para a direita"
    V = "Sinal V"
    HANG_LOOSE = "Hang loose"
    MAO_ABERTA = "Mao aberta"
    VARIAS_MAOS = "Varias maos"
    VARIAS_PESSOAS = "Varias pessoas"
    DESCONHECIDO = "Gesto desconhecido"


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def tamanho_palma(p: np.ndarray) -> float:
    return 0.5 * (_dist(p[0], p[9]) + _dist(p[5], p[17]))


def _estados_dedos(p: np.ndarray) -> tuple[set[str], set[str]]:
    """Dedos esticados e dobrados (exceto polegar), por distâncias relativas ao pulso."""
    esticados, dobrados = set(), set()
    for nome, (pip, ponta) in DEDOS.items():
        razao = _dist(p[ponta], p[PULSO]) / max(_dist(p[pip], p[PULSO]), 1e-6)
        if razao > FATOR_ESTICADO:
            esticados.add(nome)
        elif razao < FATOR_DOBRADO:
            dobrados.add(nome)
    return esticados, dobrados


def _polegar_esticado(p: np.ndarray, palma: float) -> bool:
    """Polegar afastado da mão (e não recolhido sobre os dedos)."""
    return _dist(p[POLEGAR_PONTA], p[INDICADOR_MCP]) / max(palma, 1e-6) > FATOR_POLEGAR_ESTICADO


def polegar_fora_da_mao(p: np.ndarray, palma: float) -> float:
    """Quanto (em palmas) a ponta do polegar sai da caixa dos outros pontos da mão; 0 = dentro."""
    outros = np.delete(p, [1, 2, 3, POLEGAR_PONTA], axis=0)  # pulso e os quatro dedos
    ponta = p[POLEGAR_PONTA]
    excesso = np.maximum(outros.min(axis=0) - ponta, ponta - outros.max(axis=0))
    return float(max(0.0, excesso.max())) / max(palma, 1e-6)


def _polegar_recolhido(p: np.ndarray, palma: float) -> bool:
    """Polegar dobrado sobre a mão. Polegar para cima/baixo ou torto não é nem recolhido nem esticado."""
    return (
        polegar_fora_da_mao(p, palma) <= POLEGAR_FORA_MAX
        and _dist(p[POLEGAR_PONTA], p[PALMA].mean(axis=0)) / max(palma, 1e-6) < FATOR_POLEGAR_RECOLHIDO
    )


def _componentes(origem: np.ndarray, destino: np.ndarray) -> tuple[float, float]:
    """(horizontal, vertical) do vetor unitário origem->destino; vertical > 0 = para cima na imagem."""
    vetor = destino - origem
    comprimento = max(float(np.linalg.norm(vetor)), 1e-6)
    return float(vetor[0]) / comprimento, -float(vetor[1]) / comprimento  # o y da imagem cresce para baixo


def classificar_pose(p: np.ndarray) -> Gesto:
    """Gesto de uma mão. `p`: landmarks (21, 2) em pixels."""
    palma = tamanho_palma(p)
    esticados, dobrados = _estados_dedos(p)
    polegar = _polegar_esticado(p, palma)

    if len(esticados) == 4:
        _, vertical = _componentes(p[PULSO], p[MEDIO_MCP])
        return Gesto.MAO_ABERTA if vertical > LIMIAR_VERTICAL else Gesto.DESCONHECIDO

    if esticados == {"indicador", "medio"} and dobrados == {"anelar", "minimo"}:
        return Gesto.V

    if esticados == {"indicador"} and dobrados == {"medio", "anelar", "minimo"}:
        _, vertical = _componentes(p[INDICADOR_MCP], p[INDICADOR_PONTA])
        if vertical > LIMIAR_VERTICAL:
            return Gesto.INDICADOR_CIMA
        if vertical < -LIMIAR_VERTICAL:
            return Gesto.INDICADOR_BAIXO
        return Gesto.DESCONHECIDO

    if esticados == {"minimo"} and dobrados == {"indicador", "medio", "anelar"}:
        return Gesto.HANG_LOOSE if polegar else Gesto.DESCONHECIDO

    if len(dobrados) == 4:
        if _polegar_recolhido(p, palma):
            return Gesto.PUNHO_FECHADO  # só um punho de verdade pousa: polegar torto é DESCONHECIDO
        if not polegar:
            return Gesto.DESCONHECIDO
        horizontal, _ = _componentes(p[POLEGAR_MCP], p[POLEGAR_PONTA])
        if horizontal > LIMIAR_HORIZONTAL:
            return Gesto.POLEGAR_DIREITA
        if horizontal < -LIMIAR_HORIZONTAL:
            return Gesto.POLEGAR_ESQUERDA
        return Gesto.DESCONHECIDO  # polegar torto, para cima ou para baixo: não é comando

    return Gesto.DESCONHECIDO


def classificar_maos(maos: list[np.ndarray]) -> Gesto:
    """Gesto do quadro todo. Só vale com exatamente uma mão: com mais, ninguém sabe de quem é o comando."""
    if not maos:
        return Gesto.NENHUMA_MAO
    if len(maos) > 1:
        return Gesto.VARIAS_MAOS
    return classificar_pose(maos[0])

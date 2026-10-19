"""Onde recortar a mão, a partir do esqueleto do corpo (só numpy, sem MediaPipe).

Com a câmera do drone a mão é pequena demais para o detector de palma do MediaPipe, que olha o
quadro inteiro reduzido. Por isso o detector (`visao_mao.py`) primeiro acha a *pessoa* com o
MediaPipe Pose, e este módulo diz, a partir dos 33 pontos do corpo, que pedaço do quadro original
contém cada mão levantada. O modelo de mão roda só nesses recortes ("zoom por software").

Testado em `test/test_recorte_mao.py`.
"""

from typing import NamedTuple

import numpy as np

# Índices do MediaPipe Pose (33 pontos).
OMBRO_E, OMBRO_D = 11, 12
COTOVELO_E, COTOVELO_D = 13, 14
PULSO_E, PULSO_D = 15, 16
MINIMO_E, MINIMO_D = 17, 18
INDICADOR_E, INDICADOR_D = 19, 20
QUADRIL_E, QUADRIL_D = 23, 24
LADOS = ((PULSO_E, COTOVELO_E, MINIMO_E, INDICADOR_E), (PULSO_D, COTOVELO_D, MINIMO_D, INDICADOR_D))

VISIBILIDADE_MIN = 0.5  # pontos do Pose com visibilidade abaixo disso são ignorados
# Lado do recorte em larguras de ombro. A mão inteira (do pulso à ponta dos dedos) mede uns 0,5
# ombro; 1,1 deixa margem para a mão aberta e para o erro do Pose na posição da mão.
LADO_EM_OMBROS = 1.1
LADO_MIN_PX = 48.0  # recortes menores que isso não têm detalhe nenhum
# Sem os quadris visíveis, a cintura é estimada a esta distância (em larguras de ombro) abaixo dos ombros.
TRONCO_EM_OMBROS = 1.5
DUPLICADA_EM_PALMAS = 0.5  # duas mãos com pulsos mais perto que isso (em palmas) são a mesma


class Recorte(NamedTuple):
    """Quadrado no quadro original: centro (cx, cy) e lado, em pixels."""

    cx: float
    cy: float
    lado: float


def _visivel(vis: np.ndarray, *indices: int) -> bool:
    return all(vis[i] >= VISIBILIDADE_MIN for i in indices)


def recortes_maos(corpo: np.ndarray, visibilidade: np.ndarray) -> list[Recorte] | None:
    """Recortes das mãos *levantadas* (pulso acima do meio do tronco) de uma pessoa.

    `corpo`: pontos (33, 2) do Pose em pixels; `visibilidade`: (33,). Mão abaixada (braço solto
    ao lado do corpo, que costuma parecer um punho) não vira recorte, então não comanda nada.

    None = não dá para avaliar (ombros fora da imagem, por exemplo com a pessoa muito perto da
    câmera): quem chama deve procurar a mão no quadro inteiro. [] = mãos abaixadas.
    """
    if not _visivel(visibilidade, OMBRO_E, OMBRO_D):
        return None
    ombros = (corpo[OMBRO_E] + corpo[OMBRO_D]) / 2
    largura_ombros = float(np.linalg.norm(corpo[OMBRO_E] - corpo[OMBRO_D]))
    if largura_ombros < 1.0:
        return None
    if _visivel(visibilidade, QUADRIL_E, QUADRIL_D):
        cintura_y = float((corpo[QUADRIL_E][1] + corpo[QUADRIL_D][1]) / 2)
    else:
        cintura_y = float(ombros[1]) + TRONCO_EM_OMBROS * largura_ombros
    meio_tronco_y = (float(ombros[1]) + cintura_y) / 2  # o y da imagem cresce para baixo

    recortes = []
    for pulso, cotovelo, minimo, indicador in LADOS:
        if not _visivel(visibilidade, pulso) or corpo[pulso][1] > meio_tronco_y:
            continue
        if _visivel(visibilidade, minimo, indicador):
            centro = (corpo[minimo] + corpo[indicador]) / 2
        elif _visivel(visibilidade, cotovelo):
            centro = corpo[pulso] + 0.35 * (corpo[pulso] - corpo[cotovelo])  # prolonga o antebraço
        else:
            centro = corpo[pulso]
        lado = max(LADO_EM_OMBROS * largura_ombros, LADO_MIN_PX)
        recortes.append(Recorte(float(centro[0]), float(centro[1]), lado))
    return recortes


def para_quadro(pontos_normalizados: np.ndarray, recorte: Recorte) -> np.ndarray:
    """Pontos (N, 2) normalizados [0, 1] no recorte -> pixels no quadro original."""
    origem = np.array([recorte.cx - recorte.lado / 2, recorte.cy - recorte.lado / 2])
    return origem + np.asarray(pontos_normalizados, dtype=float) * recorte.lado


def remover_duplicadas(maos: list[np.ndarray]) -> list[np.ndarray]:
    """Tira mãos repetidas (a mesma mão achada em dois recortes que se sobrepõem)."""
    unicas: list[np.ndarray] = []
    for mao in maos:
        palma = float(np.linalg.norm(mao[0] - mao[9]))  # pulso -> base do dedo médio
        if all(np.linalg.norm(mao[0] - outra[0]) > DUPLICADA_EM_PALMAS * max(palma, 1.0) for outra in unicas):
            unicas.append(mao)
    return unicas

"""Classificação das poses com mãos sintéticas (21 landmarks desenhados à mão, sem MediaPipe).

A mão-base aponta para cima na imagem (y cresce para baixo). Os números são proporções de uma
mão real; o que importa aqui é a *topologia* (qual dedo está esticado, para onde aponta).
"""

import math

import numpy as np
import pytest

from ponte_visao_drone.poses_mao import Gesto, classificar_maos, classificar_pose

BASE_DEDOS = {"indicador": (5, -0.35, -0.9), "medio": (9, -0.1, -1.0), "anelar": (13, 0.15, -0.95), "minimo": (17, 0.38, -0.8)}


def _dedo(primeiro: int, x: float, y: float, esticado: bool) -> list[tuple[int, float, float]]:
    """MCP, PIP, DIP e ponta; fechado = a ponta volta para a palma."""
    if esticado:
        return [(primeiro, x, y), (primeiro + 1, x, y - 0.4), (primeiro + 2, x, y - 0.65), (primeiro + 3, x, y - 0.85)]
    return [(primeiro, x, y), (primeiro + 1, x, y - 0.35), (primeiro + 2, x, y - 0.1), (primeiro + 3, x, y + 0.1)]


def mao(esticados=(), polegar="recolhido", angulo_mao=0.0, centro=(320.0, 240.0), escala=100.0) -> np.ndarray:
    """Landmarks (21, 2) em pixels.

    polegar: "recolhido" ou o ângulo (graus, 0 = direita da imagem, 90 = cima) em que ele aponta.
    angulo_mao: rotação da mão inteira, em graus anti-horários na imagem.
    """
    pontos = {0: (0.0, 0.0), 1: (-0.25, -0.25), 2: (-0.5, -0.5)}
    for nome, (primeiro, x, y) in BASE_DEDOS.items():
        for i, px, py in _dedo(primeiro, x, y, nome in esticados):
            pontos[i] = (px, py)
    if polegar == "recolhido":
        pontos[3], pontos[4] = (-0.35, -0.7), (-0.1, -0.65)
    elif isinstance(polegar, tuple):  # ponta do polegar numa posição dada (coordenadas da mão-base)
        pontos[3], pontos[4] = tuple((np.array(pontos[2]) + np.array(polegar)) / 2), polegar
    else:
        a = math.radians(polegar)
        direcao = np.array([math.cos(a), -math.sin(a)])
        espelhar = direcao[0] > 0.05  # a mão-base tem o polegar à esquerda; apontar para a direita é a outra mão
        if espelhar:
            direcao[0] = -direcao[0]
        base = np.array(pontos[2])
        pontos[3], pontos[4] = tuple(base + 0.3 * direcao), tuple(base + 0.55 * direcao)
        if espelhar:
            pontos = {i: (-x, y) for i, (x, y) in pontos.items()}

    p = np.array([pontos[i] for i in range(21)])
    r = math.radians(angulo_mao)
    giro = np.array([[math.cos(r), math.sin(r)], [-math.sin(r), math.cos(r)]])  # anti-horário com y para baixo
    return (p @ giro.T) * escala + np.array(centro)


def classe(p: np.ndarray) -> Gesto:
    return classificar_pose(p)


def test_indicador_para_cima_e_para_baixo():
    assert classe(mao({"indicador"})) is Gesto.INDICADOR_CIMA
    assert classe(mao({"indicador"}, angulo_mao=180)) is Gesto.INDICADOR_BAIXO
    assert classe(mao({"indicador"}, angulo_mao=15)) is Gesto.INDICADOR_CIMA  # tolera inclinação


def test_indicador_de_lado_nao_e_comando():
    assert classe(mao({"indicador"}, angulo_mao=90)) is Gesto.DESCONHECIDO
    assert classe(mao({"indicador"}, angulo_mao=-90)) is Gesto.DESCONHECIDO


def test_indicador_ignora_o_polegar():
    assert classe(mao({"indicador"}, polegar=180)) is Gesto.INDICADOR_CIMA


def test_polegar_para_os_lados_com_punho_fechado():
    assert classe(mao(polegar=0)) is Gesto.POLEGAR_DIREITA
    assert classe(mao(polegar=180)) is Gesto.POLEGAR_ESQUERDA
    assert classe(mao(polegar=20)) is Gesto.POLEGAR_DIREITA  # um pouco inclinado ainda vale


def test_polegar_para_cima_ou_baixo_nao_e_comando():
    assert classe(mao(polegar=90)) is Gesto.DESCONHECIDO
    assert classe(mao(polegar=-90)) is Gesto.DESCONHECIDO
    assert classe(mao(polegar=60)) is Gesto.DESCONHECIDO


def test_punho_fechado_com_polegar_recolhido():
    assert classe(mao()) is Gesto.PUNHO_FECHADO


def test_mao_fechada_com_polegar_por_cima_dos_dedos():
    # Polegar atravessado sobre os dedos dobrados: a ponta fica dentro da mão, mas a ~0,55 palma do
    # centro, o que a regra antiga (0,5) recusava. Era o jeito mais comum de fechar a mão na gravação.
    assert classe(mao(polegar=(0.25, -1.15))) is Gesto.PUNHO_FECHADO
    assert classe(mao(polegar=(0.25, -1.15), angulo_mao=25)) is Gesto.PUNHO_FECHADO


def test_polegar_saindo_da_mao_nao_e_mao_fechada():
    # Ponta do polegar logo acima dos dedos dobrados (joinha curto): sai da caixa da mão.
    assert classe(mao(polegar=(-0.45, -1.6))) is not Gesto.PUNHO_FECHADO
    # Para o lado, rente à mão mas fora dela.
    assert classe(mao(polegar=(-0.75, -0.6))) is not Gesto.PUNHO_FECHADO


def test_v_nao_depende_do_lado_da_tela():
    for x in (100.0, 320.0, 540.0):
        assert classe(mao({"indicador", "medio"}, centro=(x, 240.0))) is Gesto.V


def test_hang_loose():
    assert classe(mao({"minimo"}, polegar=180)) is Gesto.HANG_LOOSE
    assert classe(mao({"minimo"}, polegar=180, angulo_mao=90)) is Gesto.HANG_LOOSE  # de lado também
    assert classe(mao({"minimo"})) is Gesto.DESCONHECIDO  # mínimo sem o polegar


def test_mao_aberta_so_com_os_dedos_para_cima():
    assert classe(mao({"indicador", "medio", "anelar", "minimo"}, polegar=150)) is Gesto.MAO_ABERTA
    assert classe(mao({"indicador", "medio", "anelar", "minimo"}, polegar=150, angulo_mao=20)) is Gesto.MAO_ABERTA
    assert classe(mao({"indicador", "medio", "anelar", "minimo"}, angulo_mao=90)) is Gesto.DESCONHECIDO
    assert classe(mao({"indicador", "medio", "anelar", "minimo"}, angulo_mao=180)) is Gesto.DESCONHECIDO


@pytest.mark.parametrize("esticados", [{"medio"}, {"anelar"}, {"indicador", "anelar"}, {"medio", "anelar", "minimo"}])
def test_outras_combinacoes_sao_desconhecidas(esticados):
    assert classe(mao(esticados)) is Gesto.DESCONHECIDO


def test_varias_maos_nao_sao_comando():
    punho = mao()
    assert classificar_maos([]) is Gesto.NENHUMA_MAO
    assert classificar_maos([punho]) is Gesto.PUNHO_FECHADO
    assert classificar_maos([punho, mao({"indicador"}, centro=(500.0, 240.0))]) is Gesto.VARIAS_MAOS

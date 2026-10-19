"""Recortes da mão a partir de um esqueleto sintético do Pose (33 pontos, em pixels)."""

import numpy as np

from ponte_visao_drone.recorte_mao import (
    COTOVELO_D,
    INDICADOR_D,
    INDICADOR_E,
    LADO_EM_OMBROS,
    LADO_MIN_PX,
    MINIMO_D,
    MINIMO_E,
    PULSO_D,
    PULSO_E,
    QUADRIL_D,
    QUADRIL_E,
    Recorte,
    para_quadro,
    recortes_maos,
    remover_duplicadas,
)


def corpo(pulso_e=(260.0, 400.0), pulso_d=(420.0, 400.0), largura_ombros=80.0, quadris=True):
    """Pessoa de frente: ombros em y=200, quadris em y=360; mãos com indicador/mínimo logo acima do pulso."""
    p = np.zeros((33, 2))
    vis = np.ones(33)
    p[11], p[12] = (340 - largura_ombros / 2, 200), (340 + largura_ombros / 2, 200)
    p[13], p[14] = (280, 300), (400, 300)  # cotovelos
    for pulso, minimo, indicador, xy in ((PULSO_E, MINIMO_E, INDICADOR_E, pulso_e), (PULSO_D, MINIMO_D, INDICADOR_D, pulso_d)):
        p[pulso] = xy
        p[minimo] = (xy[0] + 8, xy[1] - 20)
        p[indicador] = (xy[0] - 8, xy[1] - 20)
    p[QUADRIL_E], p[QUADRIL_D] = (310, 360), (370, 360)
    if not quadris:
        vis[[QUADRIL_E, QUADRIL_D]] = 0.0
    return p, vis


def test_bracos_abaixados_nao_viram_recorte():
    assert recortes_maos(*corpo()) == []


def test_mao_levantada_vira_recorte_centrado_entre_indicador_e_minimo():
    (r,) = recortes_maos(*corpo(pulso_d=(420.0, 150.0)))
    assert (r.cx, r.cy) == (420.0, 130.0)
    assert r.lado == LADO_EM_OMBROS * 80.0


def test_duas_maos_levantadas():
    assert len(recortes_maos(*corpo(pulso_e=(260.0, 150.0), pulso_d=(420.0, 150.0)))) == 2


def test_limite_e_o_meio_do_tronco():
    # Ombros em 200 e quadris em 360: meio do tronco em 280.
    assert len(recortes_maos(*corpo(pulso_d=(420.0, 270.0)))) == 1
    assert recortes_maos(*corpo(pulso_d=(420.0, 290.0))) == []


def test_sem_quadris_estima_a_cintura_pelos_ombros():
    # Cintura estimada em 200 + 1,5 * 80 = 320, meio do tronco em 260.
    assert len(recortes_maos(*corpo(pulso_d=(420.0, 250.0), quadris=False))) == 1
    assert recortes_maos(*corpo(pulso_d=(420.0, 270.0), quadris=False)) == []


def test_pontos_invisiveis():
    p, vis = corpo(pulso_d=(420.0, 150.0))
    vis[PULSO_D] = 0.1
    assert recortes_maos(p, vis) == []  # pulso escondido: nada
    p, vis = corpo(pulso_d=(420.0, 150.0))
    vis[[MINIMO_D, INDICADOR_D]] = 0.1
    (r,) = recortes_maos(p, vis)  # sem os dedos do Pose: prolonga o antebraço
    np.testing.assert_allclose((r.cx, r.cy), np.array([420.0, 150.0]) + 0.35 * (np.array([420.0, 150.0]) - p[COTOVELO_D]))
    p, vis = corpo(pulso_d=(420.0, 150.0))
    vis[[11, 12]] = 0.1
    assert recortes_maos(p, vis) is None  # sem ombros não dá para avaliar: o detector usa o quadro inteiro


def test_pessoa_longe_tem_recorte_minimo():
    (r,) = recortes_maos(*corpo(pulso_d=(345.0, 150.0), largura_ombros=10.0))
    assert r.lado == LADO_MIN_PX


def test_para_quadro():
    r = Recorte(cx=100.0, cy=50.0, lado=40.0)
    np.testing.assert_allclose(para_quadro(np.array([[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]]), r),
                               [[80.0, 30.0], [100.0, 50.0], [120.0, 70.0]])


def test_remover_duplicadas():
    mao = np.zeros((21, 2))
    mao[9] = (0.0, -40.0)  # palma de 40 px
    perto, longe = mao + 10.0, mao + 100.0
    assert len(remover_duplicadas([mao, perto])) == 1
    assert len(remover_duplicadas([mao, longe])) == 2

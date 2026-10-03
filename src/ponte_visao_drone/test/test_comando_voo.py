import math

from ponte_visao_drone.comando_voo import (
    PARADO,
    RETORNO_S,
    VELOCIDADE_HORIZONTAL,
    VELOCIDADE_VERTICAL,
    VELOCIDADE_YAW,
    ZEROS,
    FiltroRetorno,
    Suavizador,
    alvo_do_gesto,
    limitar,
)


def test_velocidades_maximas_sao_20_por_cento():
    assert VELOCIDADE_HORIZONTAL == VELOCIDADE_VERTICAL == VELOCIDADE_YAW == 0.2


def test_limitar_corta_e_trata_nan():
    assert limitar(5) == 1.0
    assert limitar(-5) == -1.0
    assert limitar(float("nan")) == 0.0
    assert limitar(float("inf")) == 0.0


def test_mapeamento_segue_rep103():
    assert alvo_do_gesto("MAO_ESQUERDA") == (0, 0.2, 0, 0)  # esquerda = y+
    assert alvo_do_gesto("MAO_DIREITA") == (0, -0.2, 0, 0)
    assert alvo_do_gesto("MAO_SUBINDO") == (0, 0, 0.2, 0)
    assert alvo_do_gesto("MAO_DESCENDO") == (0, 0, -0.2, 0)
    assert alvo_do_gesto("MAO_APROXIMANDO") == (0.2, 0, 0, 0)
    assert alvo_do_gesto("MAO_AFASTANDO") == (-0.2, 0, 0, 0)
    assert alvo_do_gesto("V_ESQUERDA") == (0, 0, 0, 0.2)  # anti-horário = yaw+
    assert alvo_do_gesto("V_DIREITA") == (0, 0, 0, -0.2)


def test_gestos_de_parada_e_desconhecidos_dao_zeros():
    for nome in ("NENHUMA_MAO", "PUNHO_FECHADO", PARADO, "DESCONHECIDO", "POLEGAR_CIMA", "xyz"):
        assert alvo_do_gesto(nome) == ZEROS


def test_filtro_ignora_o_retorno_da_mao():
    f = FiltroRetorno()
    assert f.filtrar("MAO_SUBINDO", 10.0) == "MAO_SUBINDO"
    assert f.filtrar(PARADO, 10.3) == PARADO
    assert f.filtrar("MAO_DESCENDO", 10.5) == PARADO  # mão voltando
    assert f.filtrar("MAO_DESCENDO", 10.0 + RETORNO_S - 0.01) == PARADO
    assert f.filtrar("MAO_DESCENDO", 10.0 + RETORNO_S + 0.01) == "MAO_DESCENDO"  # pausa longa: é comando


def test_filtro_nao_afeta_outros_eixos_nem_o_mesmo_gesto():
    f = FiltroRetorno()
    assert f.filtrar("MAO_SUBINDO", 1.0) == "MAO_SUBINDO"
    assert f.filtrar("MAO_ESQUERDA", 1.1) == "MAO_ESQUERDA"
    assert f.filtrar("V_ESQUERDA", 1.2) == "V_ESQUERDA"
    f = FiltroRetorno()
    f.filtrar("MAO_SUBINDO", 1.0)
    assert f.filtrar("MAO_SUBINDO", 1.3) == "MAO_SUBINDO"


def test_filtro_reiniciar():
    f = FiltroRetorno()
    f.filtrar("MAO_SUBINDO", 1.0)
    f.reiniciar()
    assert f.filtrar("MAO_DESCENDO", 1.1) == "MAO_DESCENDO"


def test_rampa_sobe_em_cerca_de_300ms_e_desce_em_150ms():
    s = Suavizador()
    alvo = (0.2, 0.0, 0.0, 0.0)
    t = 0.0
    while s.atual[0] < 0.2 - 1e-9:
        s.passo(alvo, 0.05)
        t += 0.05
    assert 0.25 <= t <= 0.35
    t = 0.0
    while s.atual[0] > 1e-9:
        s.passo(ZEROS, 0.05)
        t += 0.05
    assert 0.10 <= t <= 0.20


def test_inverter_sentido_passa_por_zero():
    s = Suavizador()
    s.atual = (0.0, 0.2, 0.0, 0.0)
    visto = []
    for _ in range(20):
        visto.append(s.passo((0.0, -0.2, 0.0, 0.0), 0.05)[1])
    assert min(abs(v) for v in visto[:5]) < 1e-9
    assert math.isclose(visto[-1], -0.2)
    assert all(a >= b for a, b in zip(visto, visto[1:]))  # nunca dá um salto


def test_suavizador_limita_alvo_invalido():
    s = Suavizador()
    for _ in range(100):
        saida = s.passo((9.0, float("nan"), -9.0, 0.0), 0.05)
    assert saida == (1.0, 0.0, -1.0, 0.0)


def test_reiniciar_zera():
    s = Suavizador()
    s.passo((0.2, 0, 0, 0), 0.5)
    s.reiniciar()
    assert s.atual == ZEROS

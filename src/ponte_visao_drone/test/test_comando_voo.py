import math

from ponte_visao_drone.comando_voo import (
    PARADO,
    VELOCIDADE_HORIZONTAL,
    VELOCIDADE_VERTICAL,
    VELOCIDADE_YAW,
    ZEROS,
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
    # Drone de frente para a pessoa: a direita dela é a esquerda do drone (y+).
    assert alvo_do_gesto("POLEGAR_DIREITA") == (0, 0.2, 0, 0)
    assert alvo_do_gesto("POLEGAR_ESQUERDA") == (0, -0.2, 0, 0)
    assert alvo_do_gesto("INDICADOR_CIMA") == (0, 0, 0.2, 0)
    assert alvo_do_gesto("INDICADOR_BAIXO") == (0, 0, -0.2, 0)
    assert alvo_do_gesto("HANG_LOOSE") == (0.1, 0, 0, 0)  # frente = x+, metade da velocidade
    assert alvo_do_gesto("MAO_ABERTA") == (-0.2, 0, 0, 0)
    assert alvo_do_gesto("V") == (0, 0, 0, 0.2)  # sempre anti-horário = yaw+


def test_gestos_sem_comando_e_desconhecidos_dao_zeros():
    # O punho fechado só pousa (por RPC, no nó); como velocidade é zero.
    for nome in ("NENHUMA_MAO", "PUNHO_FECHADO", PARADO, "DESCONHECIDO", "VARIAS_MAOS", "VARIAS_PESSOAS", "MAO_SUBINDO", "xyz"):
        assert alvo_do_gesto(nome) == ZEROS


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

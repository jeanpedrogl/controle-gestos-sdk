import numpy as np

from ponte_visao_drone.comando_voo import ZEROS
from ponte_visao_drone.missao_fase3 import (
    ALTURA_ALVO,
    BUSCA_MAX_S,
    CONTAGEM_S,
    COMANDO_ESTAVEL_S,
    ENQUADRADO_S,
    GIMBAL_NA_BASE_GRAUS,
    GIMBAL_NORMAL_GRAUS,
    SUBIDA_S,
    Entrada,
    Estado,
    MissaoFase3,
    Pessoa,
    pessoa_do_corpo,
)

PRONTO = dict(telemetria_ok=True, drone_conectado=True, stick_assinado=True)


def e(agora, **kw):
    return Entrada(agora=agora, **{**PRONTO, **kw})


def ate_gestos(m: MissaoFase3) -> float:
    """Leva a missão do chão até GESTOS; devolve o instante final."""
    m.passo(e(0.0, voando=False, stick_mode=False))
    m.passo(e(SUBIDA_S, voando=True, stick_mode=True))
    perto = Pessoa(cx=0.0, altura=ALTURA_ALVO)
    t = SUBIDA_S + 0.1
    m.passo(e(t, voando=True, stick_mode=True, pessoa=perto))
    m.passo(e(t + 0.1, voando=True, stick_mode=True, pessoa=perto))
    t += 0.2 + ENQUADRADO_S
    m.passo(e(t, voando=True, stick_mode=True, pessoa=perto))
    return t


def test_sem_telemetria_so_espera():
    m = MissaoFase3(contagem_s=0)
    s = m.passo(Entrada(agora=0.0, telemetria_ok=False))
    assert m.estado is Estado.ESPERANDO_DRONE and s.acao is None and not s.armado


def test_drone_pronto_arma_e_decola_sozinho():
    m = MissaoFase3(contagem_s=0)
    s = m.passo(e(0.0, voando=False, stick_mode=False))
    assert s.acao == "decolar" and s.armado and m.estado is Estado.DECOLANDO


def test_espera_a_subida_antes_de_procurar():
    m = MissaoFase3(contagem_s=0)
    m.passo(e(0.0, voando=False))
    m.passo(e(SUBIDA_S - 1, voando=True, stick_mode=True))
    assert m.estado is Estado.DECOLANDO
    m.passo(e(SUBIDA_S, voando=True, stick_mode=True))
    assert m.estado is Estado.PROCURANDO_OPERADOR


def test_procura_gira_anti_horario_e_passa_aos_gestos_no_limite():
    m = MissaoFase3(contagem_s=0)
    m.passo(e(0.0, voando=False))
    m.passo(e(SUBIDA_S, voando=True, stick_mode=True))
    s = m.passo(e(SUBIDA_S + 1, voando=True, stick_mode=True))
    assert s.velocidade[3] > 0 and s.velocidade[:3] == (0, 0, 0)
    s = m.passo(e(SUBIDA_S + BUSCA_MAX_S + 1, voando=True, stick_mode=True))
    assert m.estado is Estado.GESTOS and s.armado  # não desiste: o operador ainda pode chegar perto


def test_vaivem_entre_procura_e_aproximacao_tem_limite_total():
    m = MissaoFase3(contagem_s=0)
    m.passo(e(0.0, voando=False))
    m.passo(e(SUBIDA_S, voando=True, stick_mode=True))
    t, longe = SUBIDA_S, Pessoa(cx=0.0, altura=0.2)
    while t < SUBIDA_S + BUSCA_MAX_S + 5 and m.estado is not Estado.GESTOS:
        m.passo(e(t, voando=True, stick_mode=True, pessoa=longe))  # aparece...
        m.passo(e(t + 2, voando=True, stick_mode=True))  # ...e some por 2 s: volta a procurar
        t += 3
    assert m.estado is Estado.GESTOS and t <= SUBIDA_S + BUSCA_MAX_S + 3


def test_aproximacao_gira_para_a_pessoa_e_so_avanca_centrada():
    m = MissaoFase3(contagem_s=0)
    m.passo(e(0.0, voando=False))
    m.passo(e(SUBIDA_S, voando=True, stick_mode=True))
    t = SUBIDA_S + 0.1
    m.passo(e(t, voando=True, stick_mode=True, pessoa=Pessoa(0.8, 0.2)))
    assert m.estado is Estado.APROXIMANDO
    s = m.passo(e(t + 0.05, voando=True, stick_mode=True, pessoa=Pessoa(0.8, 0.2)))
    assert s.velocidade[3] < 0  # pessoa à direita: horário (yaw-)
    assert s.velocidade[0] == 0  # longe do centro: não avança
    s = m.passo(e(t + 0.1, voando=True, stick_mode=True, pessoa=Pessoa(0.0, 0.2)))
    assert s.velocidade[0] > 0


def test_reconhece_operador_e_mostra_no_terminal():
    m = MissaoFase3(contagem_s=0)
    ate_gestos(m)
    assert m.estado is Estado.GESTOS


def test_gestos_viram_velocidade_e_aparecem_no_terminal():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    s = m.passo(e(t + 0.05, voando=True, stick_mode=True, gesto="INDICADOR_CIMA"))
    assert s.velocidade[2] > 0  # a velocidade não espera
    assert s.mensagens == []  # o terminal espera o gesto se manter
    s = m.passo(e(t + 0.06 + COMANDO_ESTAVEL_S, voando=True, stick_mode=True, gesto="INDICADOR_CIMA"))
    assert "COMANDO:INDICADOR_CIMA -> subir" in s.mensagens
    s = m.passo(e(t + 1, voando=True, stick_mode=True, gesto="INDICADOR_CIMA"))
    assert s.mensagens == []  # só uma vez por gesto


def test_gesto_que_pisca_nao_vai_para_o_terminal_e_repeticao_volta_a_aparecer():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    msgs = []
    for i in range(10):  # alterna a cada quadro: nunca fica estável
        msgs += m.passo(e(t + i * 0.05, voando=True, stick_mode=True,
                          gesto="V" if i % 2 else "INDICADOR_CIMA")).mensagens
    assert not any(msg.startswith("COMANDO") for msg in msgs)
    for i, gesto in enumerate(["V"] * 8 + ["NENHUMA_MAO"] * 8 + ["V"] * 8):
        msgs += m.passo(e(t + 1 + i * 0.05, voando=True, stick_mode=True, gesto=gesto)).mensagens
    v = "COMANDO:V -> girar no sentido anti-horario"
    assert msgs.count(v) == 2 and not any("NENHUMA_MAO" in msg for msg in msgs)


def test_punho_pousa_e_conta_a_base():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    s = m.passo(e(t + 0.05, voando=True, stick_mode=True, punho=True))
    assert s.acao == "pousar" and m.estado is Estado.POUSANDO
    m.passo(e(t + 5, voando=False, stick_mode=True))
    assert m.estado is Estado.NA_BASE and m.bases_visitadas == 1


def test_stick_desligado_em_voo_aborta_e_desarma():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    s = m.passo(e(t + 0.05, voando=True, stick_mode=False, gesto="INDICADOR_CIMA"))
    assert m.estado is Estado.ABORTADO and not s.armado and s.velocidade == ZEROS


def test_telemetria_perdida_aborta():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    s = m.passo(Entrada(agora=t + 0.05, telemetria_ok=False))
    assert m.estado is Estado.ABORTADO and not s.armado


def test_abortado_nao_volta_sozinho():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    m.passo(Entrada(agora=t + 0.05, telemetria_ok=False))
    s = m.passo(e(t + 1, voando=True, stick_mode=True, gesto="INDICADOR_CIMA", punho=True))
    assert m.estado is Estado.ABORTADO and s.acao is None and s.velocidade == ZEROS


def pousar_na_base(m: MissaoFase3, t: float) -> float:
    """De GESTOS (ou VOLTANDO), fecha o punho e toca o chão; devolve o instante final."""
    m.passo(e(t, voando=True, stick_mode=True, punho=True))
    m.passo(e(t + 4, voando=False, stick_mode=True))
    return t + 4


def decolar_da_base(m: MissaoFase3, t: float) -> tuple[float, object]:
    """De NA_BASE, fecha o punho e sobe; devolve o instante final e a Saida do punho."""
    s = m.passo(e(t, voando=False, stick_mode=True, punho=True))
    m.passo(e(t + SUBIDA_S, voando=True, stick_mode=True))
    return t + SUBIDA_S, s


def test_punho_na_base_decola_de_novo():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    s = m.passo(e(t + 1, voando=False, stick_mode=True))
    assert m.estado is Estado.NA_BASE and s.acao is None  # sem punho: espera
    t, s = decolar_da_base(m, t + 2)
    assert s.acao == "decolar" and s.armado
    assert m.estado is Estado.GESTOS  # o operador já foi reconhecido: não procura de novo


def test_na_base_sem_saber_se_esta_no_chao_nao_decola():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    s = m.passo(e(t + 1, voando=None, stick_mode=True, punho=True))
    assert m.estado is Estado.NA_BASE and s.acao is None


def test_panico_na_base_aborta_e_punho_nao_decola():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    s = m.passo(e(t + 1, voando=False, stick_mode=False, punho=True))
    assert m.estado is Estado.ABORTADO and s.acao is None and not s.armado


def test_decolagem_pelo_rc_na_base_aborta():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    s = m.passo(e(t + 1, voando=True, stick_mode=True))
    assert m.estado is Estado.ABORTADO and not s.armado


def test_decolagem_recusada_volta_para_a_base():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    m.passo(e(t + 1, voando=False, stick_mode=True, punho=True))
    m.passo(e(t + 20, voando=False, stick_mode=True))
    assert m.estado is Estado.NA_BASE and m.bases_visitadas == 1


def test_missao_completa_seis_bases_volta_e_termina():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    for n in range(1, 7):
        t = pousar_na_base(m, t + 1)
        assert m.estado is Estado.NA_BASE and m.bases_visitadas == n
        if n < 6:
            t, _ = decolar_da_base(m, t + 1)
            assert m.estado is Estado.GESTOS
    s = m.passo(e(t + 1, voando=False, stick_mode=True, voltar=True))  # dois punhos na 6ª base
    assert s.acao == "decolar"
    m.passo(e(t + 1 + SUBIDA_S, voando=True, stick_mode=True))
    assert m.estado is Estado.VOLTANDO
    m.passo(e(t + 10, voando=True, stick_mode=True, punho=True))
    s = m.passo(e(t + 14, voando=False, stick_mode=True))
    assert m.estado is Estado.FIM and m.bases_visitadas == 6 and not s.armado


def test_seis_pousos_nao_viram_volta_sozinhos():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    for _ in range(6):
        t = pousar_na_base(m, t + 1)
        t, _ = decolar_da_base(m, t + 1)
    assert m.estado is Estado.GESTOS  # um pouso pode ter sido repetido: quem decide é o operador


def test_dois_punhos_antes_da_primeira_base_sao_ignorados():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    s = m.passo(e(t + 1, voando=True, stick_mode=True, voltar=True))
    assert m.estado is Estado.GESTOS and any("ignorados" in msg for msg in s.mensagens)


def test_dois_punhos_em_voo_voltam_e_de_novo_cancelam():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    t, _ = decolar_da_base(m, t + 1)
    m.passo(e(t + 1, voando=True, stick_mode=True, voltar=True))
    assert m.estado is Estado.VOLTANDO
    m.passo(e(t + 4, voando=True, stick_mode=True, voltar=True))
    assert m.estado is Estado.GESTOS
    s = m.passo(e(t + 6, voando=True, stick_mode=True, punho=True))  # em GESTOS: pouso comum
    m.passo(e(t + 10, voando=False, stick_mode=True))
    assert s.acao == "pousar" and m.estado is Estado.NA_BASE and m.bases_visitadas == 2


def test_panico_entre_o_pedido_e_a_subida_cancela_a_decolagem():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    m.passo(e(t + 1, voando=False, stick_mode=True, punho=True))
    assert m.estado is Estado.DECOLANDO
    s = m.passo(e(t + 1.5, voando=False, stick_mode=False))
    assert m.estado is Estado.ABORTADO and s.acao == "cancelar_decolagem" and not s.armado


def test_primeira_decolagem_tolera_o_stick_ainda_desligado():
    m = MissaoFase3(contagem_s=0)
    m.passo(e(0.0, voando=False, stick_mode=False))
    m.passo(e(1.0, voando=False, stick_mode=False))  # o celular ainda está ligando o stick mode
    assert m.estado is Estado.DECOLANDO


def test_punho_na_base_aparece_no_terminal():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    msgs = []
    for i in range(10):
        msgs += m.passo(e(t + 1 + i * 0.05, voando=False, stick_mode=True, gesto="PUNHO_FECHADO")).mensagens
    assert "COMANDO:PUNHO_FECHADO -> decolar para a proxima base (segure 2 s)" in msgs


def test_camera_inclina_para_cima_na_base_e_volta_ao_decolar():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    s = m.passo(e(t + 0.05, voando=True, stick_mode=True))
    assert s.gimbal_pitch == GIMBAL_NORMAL_GRAUS  # em voo: normal
    m.passo(e(t + 1, voando=True, stick_mode=True, punho=True))
    s = m.passo(e(t + 5, voando=False, stick_mode=True))
    assert m.estado is Estado.NA_BASE and s.gimbal_pitch == GIMBAL_NA_BASE_GRAUS > 0  # pousou: sobe
    s = m.passo(e(t + 6, voando=False, stick_mode=True, punho=True))
    assert s.acao == "decolar" and s.gimbal_pitch == GIMBAL_NORMAL_GRAUS  # punho: volta junto


def test_camera_sobe_de_novo_se_a_decolagem_falhar():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    m.passo(e(t + 1, voando=False, stick_mode=True, punho=True))
    s = m.passo(e(t + 20, voando=False, stick_mode=True))
    assert m.estado is Estado.NA_BASE and s.gimbal_pitch == GIMBAL_NA_BASE_GRAUS


def test_drone_pede_confirmacao_e_o_punho_confirma():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    m.passo(e(t, voando=True, stick_mode=True, punho=True))  # pede o pouso
    s = m.passo(e(t + 3, voando=True, stick_mode=True, confirmar_pouso=True, gesto="INDICADOR_CIMA"))
    assert m.estado is Estado.POUSANDO and s.acao is None and s.velocidade == ZEROS  # gestos desligados
    assert any("CONFIRMACAO" in msg for msg in s.mensagens)
    s = m.passo(e(t + 3.05, voando=True, stick_mode=True, confirmar_pouso=True))
    assert s.mensagens == []  # avisa uma vez só
    s = m.passo(e(t + 6, voando=True, stick_mode=True, confirmar_pouso=True, punho=True))
    assert s.acao == "pousar" and m.estado is Estado.POUSANDO
    m.passo(e(t + 8, voando=False, stick_mode=True))
    assert m.estado is Estado.NA_BASE and m.bases_visitadas == 1


def test_punho_durante_a_descida_normal_nao_repete_o_pouso():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    m.passo(e(t, voando=True, stick_mode=True, punho=True))
    s = m.passo(e(t + 1, voando=True, stick_mode=True, punho=True))
    assert s.acao is None and m.estado is Estado.POUSANDO


def test_prazo_recomeca_quando_o_drone_pede_confirmacao():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    m.passo(e(t, voando=True, stick_mode=True, punho=True))
    m.passo(e(t + 15, voando=True, stick_mode=True, confirmar_pouso=True))  # pediu tarde
    m.passo(e(t + 25, voando=True, stick_mode=True, confirmar_pouso=True))  # 25 s do pouso, 10 s do pedido
    assert m.estado is Estado.POUSANDO
    m.passo(e(t + 36, voando=True, stick_mode=True, confirmar_pouso=True))  # 21 s sem confirmar
    assert m.estado is Estado.GESTOS  # volta aos gestos para reposicionar


def comandos_impressos(m: MissaoFase3, t: float, gesto: str, **kw) -> list[str]:
    """Mantém `gesto` por 0,5 s e devolve as linhas COMANDO que saíram."""
    msgs = []
    for i in range(10):
        msgs += m.passo(e(t + i * 0.05, gesto=gesto, **{"stick_mode": True, **kw})).mensagens
    return [msg for msg in msgs if msg.startswith("COMANDO:")]


def test_o_efeito_do_punho_depende_do_estado():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    assert comandos_impressos(m, t + 1, "PUNHO_FECHADO", voando=True) == [
        "COMANDO:PUNHO_FECHADO -> pousar aqui (segure 2 s)"]
    m.passo(e(t + 2, voando=True, stick_mode=True, punho=True))  # pousando
    assert comandos_impressos(m, t + 3, "PUNHO_FECHADO", voando=True, confirmar_pouso=True) == [
        "COMANDO:PUNHO_FECHADO -> confirmar o pouso (segure 2 s)"]


def test_gestos_de_movimento_nao_fazem_nada_na_base_e_o_terminal_diz_isso():
    m = MissaoFase3(contagem_s=0)
    t = pousar_na_base(m, ate_gestos(m))
    assert comandos_impressos(m, t + 1, "INDICADOR_CIMA", voando=False) == [
        "COMANDO:INDICADOR_CIMA -> nenhum (o drone esta na base)"]


def test_dois_punhos_dizem_se_valem_ou_nao():
    m = MissaoFase3(contagem_s=0)
    t = ate_gestos(m)
    assert comandos_impressos(m, t + 1, "DOIS_PUNHOS", voando=True) == [
        "COMANDO:DOIS_PUNHOS -> nenhum (nenhuma base visitada ainda)"]
    t = pousar_na_base(m, t + 2)
    assert comandos_impressos(m, t + 1, "DOIS_PUNHOS", voando=False) == [
        "COMANDO:DOIS_PUNHOS -> decolar e voltar para a base de decolagem (segure 2 s)"]


def test_reconhecimento_do_operador_aparece_no_terminal():
    m = MissaoFase3(contagem_s=0)
    m.passo(e(0.0, voando=False))
    m.passo(e(SUBIDA_S, voando=True, stick_mode=True))
    msgs, perto, t = [], Pessoa(cx=0.0, altura=ALTURA_ALVO), SUBIDA_S + 0.1
    for i in range(20):
        msgs += m.passo(e(t + i * 0.05, voando=True, stick_mode=True, pessoa=perto)).mensagens
    assert any(msg.startswith("OPERADOR DETECTADO:") for msg in msgs)
    assert any(msg.startswith("OPERADOR RECONHECIDO:") for msg in msgs)


def test_contagem_regressiva_antes_de_decolar():
    m = MissaoFase3()
    msgs = []
    for i in range(int(CONTAGEM_S * 20) + 2):
        s = m.passo(e(i * 0.05, voando=False, stick_mode=False))
        msgs += s.mensagens
        if s.acao == "decolar":
            break
    assert [x for x in msgs if x.startswith("Decolando em")] == [
        f"Decolando em {n}..." for n in range(int(CONTAGEM_S), 0, -1)]
    assert s.acao == "decolar" and m.estado is Estado.DECOLANDO
    assert i * 0.05 >= CONTAGEM_S


def test_contagem_cancela_e_recomeca_se_algo_cair():
    m = MissaoFase3()
    m.passo(e(0.0, voando=False))
    m.passo(e(2.0, voando=False))  # "Decolando em 3..."
    s = m.passo(Entrada(agora=2.5, telemetria_ok=False))
    assert m.estado is Estado.ESPERANDO_DRONE and "Contagem cancelada: sem telemetria do celular" in s.mensagens
    s = m.passo(e(3.0, voando=False))
    assert f"Decolando em {int(CONTAGEM_S)}..." in s.mensagens  # recomeça do início
    s = m.passo(e(3.0 + CONTAGEM_S - 0.1, voando=False))
    assert s.acao is None
    s = m.passo(e(3.0 + CONTAGEM_S, voando=False))
    assert s.acao == "decolar"


def corpo(cx_px: float, topo: float, escala: float) -> np.ndarray:
    """33 pontos de uma pessoa de pé: cabeça em `topo`, 1,7 m = `escala` px de altura."""
    pts = np.tile([cx_px, topo + escala / 2], (33, 1)).astype(float)
    pts[0] = [cx_px, topo]  # nariz
    pts[11], pts[12] = [cx_px + 0.12 * escala, topo + 0.2 * escala], [cx_px - 0.12 * escala, topo + 0.2 * escala]
    pts[23], pts[24] = [cx_px + 0.07 * escala, topo + 0.5 * escala], [cx_px - 0.07 * escala, topo + 0.5 * escala]
    pts[27], pts[28] = [cx_px + 0.06 * escala, topo + escala], [cx_px - 0.06 * escala, topo + escala]
    return pts


def test_pessoa_inteira_no_meio():
    p = pessoa_do_corpo(corpo(320, 100, 300), largura=640, altura=480)
    assert abs(p.cx) < 0.01 and 0.6 < p.altura < 0.7  # ~300 px de 480


def test_desfaz_o_espelho():
    # No quadro espelhado ela aparece à esquerda (x pequeno) => na câmera do drone está à direita.
    p = pessoa_do_corpo(corpo(100, 100, 300), largura=640, altura=480)
    assert p.cx > 0.6


def test_de_perto_sem_as_pernas_nao_encolhe():
    perto = corpo(320, 50, 900)  # pés bem abaixo da imagem (y até 950 num quadro de 480)
    p = pessoa_do_corpo(perto, largura=640, altura=480)
    assert p.altura == 1.0  # ombros/tronco dizem "muito perto"; a caixa cortada sozinha diria ~0,9

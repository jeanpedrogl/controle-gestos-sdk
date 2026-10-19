from ponte_visao_drone.terminal import (
    NEGRITO_AMARELO,
    NEGRITO_CIANO,
    NEGRITO_MAGENTA,
    NEGRITO_VERDE,
    colorir,
)


def test_comando_tem_tres_cores_distintas():
    linha = colorir("COMANDO:INDICADOR_CIMA -> subir", cores=True)
    assert NEGRITO_MAGENTA + "COMANDO:" in linha
    assert NEGRITO_CIANO + "INDICADOR_CIMA" in linha
    assert NEGRITO_AMARELO + "subir" in linha
    assert len({NEGRITO_MAGENTA, NEGRITO_CIANO, NEGRITO_AMARELO, NEGRITO_VERDE}) == 4


def test_efeito_com_espacos_e_parenteses_fica_inteiro_na_cor():
    linha = colorir("COMANDO:PUNHO_FECHADO -> pousar aqui (segure 2 s)", cores=True)
    assert NEGRITO_AMARELO + "pousar aqui (segure 2 s)" in linha


def test_operador_tem_o_rotulo_em_verde():
    linha = colorir("OPERADOR RECONHECIDO: centro x=+0.00", cores=True)
    assert linha.startswith(NEGRITO_VERDE + "OPERADOR RECONHECIDO:")


def test_outras_linhas_e_sem_cores_voltam_iguais():
    assert colorir("[GESTOS -> POUSANDO] pousando", cores=True) == "[GESTOS -> POUSANDO] pousando"
    assert colorir("COMANDO:V -> girar", cores=False) == "COMANDO:V -> girar"


def test_no_color_desliga(monkeypatch):
    monkeypatch.setenv("NO_COLOR", "1")
    assert colorir("COMANDO:V -> girar") == "COMANDO:V -> girar"

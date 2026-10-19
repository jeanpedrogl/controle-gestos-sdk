"""Cores ANSI para o terminal que o juiz lê na Fase 3.

Só formatação: `missao_fase3.py` produz linhas de texto puro (que também podem ir para log, rosbag
ou a janela do OpenCV, onde códigos de cor virariam lixo) e o nó passa cada uma por `colorir` na
hora de imprimir no terminal.

    COMANDO:INDICADOR_CIMA -> subir
    └ magenta └ ciano        └ amarelo

    OPERADOR RECONHECIDO: centro x=+0.00 ...
    └ verde (o rótulo), resto normal

Com a variável de ambiente NO_COLOR definida (convenção de https://no-color.org), sai sem cores.
"""

import os
import re

NEGRITO_MAGENTA = "\033[1;95m"
NEGRITO_CIANO = "\033[1;96m"
NEGRITO_AMARELO = "\033[1;93m"
NEGRITO_VERDE = "\033[1;92m"
NORMAL = "\033[0m"

_COMANDO = re.compile(r"^COMANDO:(\S+) -> (.+)$")
_OPERADOR = re.compile(r"^(OPERADOR (?:DETECTADO|RECONHECIDO):)(.*)$")


def cores_ligadas() -> bool:
    return "NO_COLOR" not in os.environ


def colorir(linha: str, cores: bool | None = None) -> str:
    """A linha com as cores do comando ou do operador; qualquer outra linha volta igual."""
    if not (cores_ligadas() if cores is None else cores):
        return linha
    comando = _COMANDO.match(linha)
    if comando:
        gesto, efeito = comando.groups()
        return (f"{NEGRITO_MAGENTA}COMANDO:{NORMAL}{NEGRITO_CIANO}{gesto}{NORMAL}"
                f" -> {NEGRITO_AMARELO}{efeito}{NORMAL}")
    operador = _OPERADOR.match(linha)
    if operador:
        rotulo, resto = operador.groups()
        return f"{NEGRITO_VERDE}{rotulo}{NORMAL}{resto}"
    return linha

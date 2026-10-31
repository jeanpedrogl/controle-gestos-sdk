"""Tira do terminal o ruído que bibliotecas nativas escrevem direto no stderr.

Durante o voo o libjpeg de dentro do OpenCV escreve "Corrupt JPEG data: ..." a cada quadro que chega
danificado do celular (comum quando o Wi-Fi engasga), e o MediaPipe/TFLite escrevem avisos INFO e
WARNING. Nada disso passa pelo `logging` do Python nem obedece a `cv2.setLogLevel`: é código C
escrevendo no descritor 2. Por isso `instalar()` troca o descritor 2 por um pipe lido por uma
thread, que repassa ao terminal toda linha que não for ruído conhecido (`eh_ruido`). Os logs do
rclpy, que também saem pelo stderr, passam inteiros; erros do MediaPipe (E/F) também.

`restaurar()` devolve o stderr original e espera a thread esvaziar o pipe: chame depois do desarme,
senão as últimas linhas (as do desarme) se perdem quando o processo termina.
"""

import os
import re
import sys
import threading

_RUIDO = re.compile(
    r"Corrupt JPEG data"  # libjpeg (OpenCV): quadro danificado no caminho celular -> computador
    r"|Premature end of JPEG file"
    r"|^INFO: Created TensorFlow Lite"  # TFLite, ao carregar os modelos
    r"|^WARNING: Logging before InitGoogle"  # absl, antes da primeira linha do MediaPipe
    r"|^[IW]\d{4} [\d:.]+ +\d+ \S+:\d+\]"  # INFO/WARNING do MediaPipe (absl/glog)
    r"|^QFontDatabase: Cannot find font directory"  # Qt do OpenCV (janela), ao abrir a janela
    r"|^Note that Qt no longer ships fonts"
    r"|^Warning: Ignoring XDG_SESSION_TYPE=wayland"
)

_estado: tuple[int, threading.Thread] | None = None  # (descritor do stderr original, thread)


def eh_ruido(linha: str) -> bool:
    return _RUIDO.search(linha) is not None


def instalar() -> None:
    """Passa o stderr do processo (descritor 2) pelo filtro. Chamar de novo não faz nada."""
    global _estado
    if _estado is not None:
        return
    sys.stderr.flush()
    original = os.dup(2)
    leitura, escrita = os.pipe()
    os.dup2(escrita, 2)
    os.close(escrita)  # a única ponta de escrita passa a ser o descritor 2
    thread = threading.Thread(target=_repassar, args=(leitura, original), name="filtro_stderr", daemon=True)
    thread.start()
    _estado = (original, thread)


def restaurar(espera_s: float = 1.0) -> None:
    """Devolve o stderr original e espera a thread repassar o que ainda estava no pipe."""
    global _estado
    if _estado is None:
        return
    original, thread = _estado
    _estado = None
    sys.stderr.flush()
    os.dup2(original, 2)  # fecha a última ponta de escrita do pipe: a thread lê o resto e vê o fim
    thread.join(espera_s)


def _repassar(leitura: int, destino: int) -> None:
    resto = b""
    while True:
        bloco = os.read(leitura, 4096)
        if not bloco:
            break
        linhas = (resto + bloco).split(b"\n")
        resto = linhas.pop()
        for linha in linhas:
            _escrever(destino, linha + b"\n")
    if resto:
        _escrever(destino, resto)
    os.close(leitura)


def _escrever(destino: int, linha: bytes) -> None:
    if eh_ruido(linha.decode(errors="replace")):
        return
    try:
        while linha:
            linha = linha[os.write(destino, linha):]
    except OSError:
        pass  # terminal fechado: a thread continua lendo, senão o pipe enche e trava quem escreve

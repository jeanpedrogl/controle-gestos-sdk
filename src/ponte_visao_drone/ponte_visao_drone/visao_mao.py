"""Detecção de gestos e movimentos da mão com OpenCV + MediaPipe.

Uso:
    with DetectorGestos() as detector:
        gesto = detector.processar(frame_bgr)
"""

import math
import time
import urllib.request
from collections import deque
from enum import Enum
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions, vision

URL_MODELO = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
CAMINHO_MODELO = Path(__file__).resolve().parent / "modelos" / "hand_landmarker.task"

FATOR_ESTICADO = 1.15  # dist(ponta, pulso) / dist(PIP, pulso) acima disso = dedo esticado
FATOR_DOBRADO = 1.0  # abaixo disso = dedo dobrado
JANELA_S = 0.5  # janela de tempo em que o movimento é medido
LIMIAR_DESLOCAMENTO = 0.10  # fração da altura da imagem
LIMIAR_ESCALA = 0.15  # log da variação do tamanho da palma
FRAMES_CONFIRMACAO = 3  # frames iguais seguidos para trocar de gesto

PULSO, POLEGAR_MCP, POLEGAR_IP, POLEGAR_PONTA = 0, 2, 3, 4
PALMA = [0, 5, 9, 13, 17]  # pulso + base de cada dedo
DEDOS = {"indicador": (6, 8), "medio": (10, 12), "anelar": (14, 16), "minimo": (18, 20)}  # (PIP, ponta)


class Gesto(Enum):
    NENHUMA_MAO = "Nenhuma mao"
    PUNHO_FECHADO = "Punho fechado"
    POLEGAR_CIMA = "Polegar para cima"
    POLEGAR_BAIXO = "Polegar para baixo"
    MAO_SUBINDO = "Mao aberta subindo"
    MAO_DESCENDO = "Mao aberta descendo"
    MAO_ESQUERDA = "Mao aberta para a esquerda"
    MAO_DIREITA = "Mao aberta para a direita"
    MAO_APROXIMANDO = "Mao aberta se aproximando"
    MAO_AFASTANDO = "Mao aberta se afastando"
    MAO_ABERTA_PARADA = "Mao aberta parada"
    V_ESQUERDA = "Sinal V a esquerda"
    V_DIREITA = "Sinal V a direita"
    DESCONHECIDO = "Gesto desconhecido"


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


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


def _direcao_polegar(p: np.ndarray, palma: float) -> int:
    """+1 se o polegar aponta para cima, -1 para baixo, 0 caso contrário."""
    vetor = p[POLEGAR_PONTA] - p[POLEGAR_MCP]
    comprimento = float(np.linalg.norm(vetor))
    if comprimento < 0.4 * palma:
        return 0
    verticalidade = -vetor[1] / comprimento  # eixo y da imagem cresce para baixo
    outros_y = np.delete(p, [POLEGAR_IP, POLEGAR_PONTA], axis=0)[:, 1]
    ponta_y, margem = p[POLEGAR_PONTA][1], 0.25 * palma
    if verticalidade > 0.6 and ponta_y < outros_y.min() - margem:
        return 1
    if verticalidade < -0.6 and ponta_y > outros_y.max() + margem:
        return -1
    return 0


class DetectorGestos:
    """Frame BGR -> landmarks -> gesto.

    Espelhe o frame da webcam (cv2.flip(frame, 1)) para que esquerda/direita
    correspondam ao ponto de vista do usuário.
    """

    def __init__(self, caminho_modelo: Path = CAMINHO_MODELO):
        if not caminho_modelo.exists():
            caminho_modelo.parent.mkdir(parents=True, exist_ok=True)
            urllib.request.urlretrieve(URL_MODELO, caminho_modelo)
        self._landmarker = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=str(caminho_modelo)),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=1,
            )
        )
        self._ultimo_ts_ms = -1
        self._historico: deque = deque()  # (t, centro, tamanho_palma) da mão aberta
        self._candidato, self._contagem = Gesto.NENHUMA_MAO, 0
        self.gesto = Gesto.NENHUMA_MAO
        self.pontos: np.ndarray | None = None  # landmarks (21, 2) em pixels do último frame

    def __enter__(self) -> "DetectorGestos":
        return self

    def __exit__(self, *_) -> None:
        self._landmarker.close()

    def processar(self, frame_bgr: np.ndarray, timestamp: float | None = None) -> Gesto:
        """Retorna o gesto estabilizado (troca só após FRAMES_CONFIRMACAO frames iguais)."""
        t = time.monotonic() if timestamp is None else timestamp
        altura, largura = frame_bgr.shape[:2]

        # O modo VIDEO exige timestamps estritamente crescentes.
        self._ultimo_ts_ms = max(int(t * 1000), self._ultimo_ts_ms + 1)
        imagem = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        landmarks = self._landmarker.detect_for_video(imagem, self._ultimo_ts_ms).hand_landmarks
        self.pontos = np.array([[lm.x * largura, lm.y * altura] for lm in landmarks[0]]) if landmarks else None

        bruto = self._classificar(largura, altura, t)
        if bruto == self._candidato:
            self._contagem += 1
        else:
            self._candidato, self._contagem = bruto, 1
        if self._contagem >= FRAMES_CONFIRMACAO:
            self.gesto = bruto
        return self.gesto

    def _classificar(self, largura: int, altura: int, t: float) -> Gesto:
        p = self.pontos
        if p is None:
            self._historico.clear()
            return Gesto.NENHUMA_MAO

        centro = p[PALMA].mean(axis=0)
        palma = 0.5 * (_dist(p[0], p[9]) + _dist(p[5], p[17]))
        esticados, dobrados = _estados_dedos(p)
        if len(esticados) == 4:
            return self._movimento(centro, palma, altura, t)

        self._historico.clear()
        if esticados == {"indicador", "medio"} and dobrados == {"anelar", "minimo"}:
            return Gesto.V_ESQUERDA if centro[0] < largura / 2 else Gesto.V_DIREITA
        if len(dobrados) == 4:
            direcao = _direcao_polegar(p, palma)
            if direcao:
                return Gesto.POLEGAR_CIMA if direcao > 0 else Gesto.POLEGAR_BAIXO
            return Gesto.PUNHO_FECHADO
        return Gesto.DESCONHECIDO

    def _movimento(self, centro: np.ndarray, palma: float, altura: int, t: float) -> Gesto:
        """Deslocamento da mão aberta na janela: centro da palma e variação do tamanho."""
        h = self._historico
        h.append((t, centro, palma))
        while t - h[0][0] > JANELA_S:
            h.popleft()
        (t0, c0, s0), (t1, c1, s1) = h[0], h[-1]
        if t1 - t0 < 0.6 * JANELA_S:
            return Gesto.MAO_ABERTA_PARADA

        dx, dy = (c1 - c0) / altura
        escala = math.log(max(s1, 1e-6) / max(s0, 1e-6))
        # Ao se aproximar, o centro também se desloca um pouco; por isso cada eixo
        # é normalizado pelo seu limiar e vence o de maior intensidade.
        pontuacoes = {
            Gesto.MAO_DIREITA if dx > 0 else Gesto.MAO_ESQUERDA: abs(dx) / LIMIAR_DESLOCAMENTO,
            Gesto.MAO_DESCENDO if dy > 0 else Gesto.MAO_SUBINDO: abs(dy) / LIMIAR_DESLOCAMENTO,
            Gesto.MAO_APROXIMANDO if escala > 0 else Gesto.MAO_AFASTANDO: abs(escala) / LIMIAR_ESCALA,
        }
        gesto, pontuacao = max(pontuacoes.items(), key=lambda item: item[1])
        return gesto if pontuacao >= 1.0 else Gesto.MAO_ABERTA_PARADA


CONEXOES_MAO = [(c.start, c.end) for c in vision.HandLandmarksConnections.HAND_CONNECTIONS]


def executar_demo(indice_camera: int = 0) -> None:
    """Mostra a webcam com o gesto reconhecido. 'q' ou ESC para sair."""
    captura = cv2.VideoCapture(indice_camera)
    ultimo = None
    try:
        with DetectorGestos() as detector:
            while True:
                ok, frame = captura.read()
                if not ok:
                    break
                frame = cv2.flip(frame, 1)
                gesto = detector.processar(frame)
                if gesto != ultimo:
                    print(gesto.name)
                    ultimo = gesto

                if detector.pontos is not None:
                    pts = detector.pontos.astype(int)
                    for a, b in CONEXOES_MAO:
                        cv2.line(frame, tuple(pts[a]), tuple(pts[b]), (255, 255, 255), 2)
                cv2.putText(frame, gesto.value, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Gestos da mao", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        captura.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    executar_demo()

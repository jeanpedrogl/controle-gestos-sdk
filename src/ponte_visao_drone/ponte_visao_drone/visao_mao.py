"""Detecção de gestos da mão com OpenCV + MediaPipe, em duas etapas ("zoom por software").

1. O MediaPipe Pose acha a *pessoa* no quadro inteiro (o corpo é grande e fácil de achar mesmo a
   alguns metros da câmera grande-angular do drone) e dá os pontos dos pulsos e das mãos.
2. Para cada mão levantada, `recorte_mao.py` diz que quadrado do quadro *original* recortar; o
   HandLandmarker roda só nesse recorte, onde a mão ocupa boa parte da imagem.

Sem nenhuma pessoa no quadro, ou com uma pessoa sem os ombros visíveis (mão bem perto da câmera,
corpo fora da imagem), o detector volta ao modo antigo: HandLandmarker no quadro inteiro. Com mais
de uma pessoa, nenhum comando.

Este módulo só cuida do MediaPipe (modelos, landmarks, estabilização no tempo); a classificação do
gesto em si está em `poses_mao.py`.

Uso:
    with DetectorGestos() as detector:
        gesto = detector.processar(frame_bgr)
"""

import time
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions, vision

from .poses_mao import PALMA, Gesto, classificar_maos
from .recorte_mao import Recorte, para_quadro, recortes_maos, remover_duplicadas

_URL_MODELOS = "https://storage.googleapis.com/mediapipe-models"
_PASTA_MODELOS = Path(__file__).resolve().parent / "modelos"
URL_MODELO = f"{_URL_MODELOS}/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
CAMINHO_MODELO = _PASTA_MODELOS / "hand_landmarker.task"
# Pose "full": meio-termo entre "lite" (mais rápido, erra mais de longe) e "heavy" (~3x mais lento).
URL_MODELO_POSE = f"{_URL_MODELOS}/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"
CAMINHO_MODELO_POSE = _PASTA_MODELOS / "pose_landmarker_full.task"

FRAMES_CONFIRMACAO = 3  # frames iguais seguidos para trocar de gesto (padrão)
TAMANHO_RECORTE = 256  # px: o recorte da mão é redimensionado para isto antes do HandLandmarker
MAX_PESSOAS = 2  # procura duas só para perceber uma segunda pessoa no quadro


def _garantir_modelo(caminho: Path, url: str) -> Path:
    """Baixa o modelo se ainda não está em `modelos/` (assim um checkout novo já roda)."""
    if not caminho.exists():
        caminho.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(url, caminho)
    return caminho


def _mp_image(bgr: np.ndarray) -> mp.Image:
    return mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))


class DetectorGestos:
    """Frame BGR -> (pessoa -> recorte da mão) -> landmarks -> gesto.

    Espelhe o frame (cv2.flip(frame, 1)) para que esquerda/direita correspondam ao ponto de
    vista de quem faz o gesto.
    """

    def __init__(
        self,
        caminho_modelo: Path = CAMINHO_MODELO,
        frames_confirmacao: int = FRAMES_CONFIRMACAO,
        caminho_modelo_pose: Path = CAMINHO_MODELO_POSE,
    ):
        self._frames_confirmacao = frames_confirmacao
        # Modo IMAGE: cada recorte é uma imagem nova (o recorte se move), o rastreamento entre
        # quadros do modo VIDEO não faria sentido aqui.
        self._maos = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=str(_garantir_modelo(caminho_modelo, URL_MODELO))),
                running_mode=vision.RunningMode.IMAGE,
                num_hands=2,  # duas para perceber uma segunda mão (ver `classificar_maos`)
            )
        )
        self._pose = vision.PoseLandmarker.create_from_options(
            vision.PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=str(_garantir_modelo(caminho_modelo_pose, URL_MODELO_POSE))),
                running_mode=vision.RunningMode.VIDEO,
                num_poses=MAX_PESSOAS,
            )
        )
        self._ultimo_ts_ms = -1
        self._candidato, self._contagem = Gesto.NENHUMA_MAO, 0
        self.gesto = Gesto.NENHUMA_MAO
        # Do último frame, para desenhar na janela:
        self.maos: list[np.ndarray] = []  # landmarks (21, 2) em pixels de cada mão
        self.corpos: list[np.ndarray] = []  # pontos (33, 2) em pixels de cada pessoa
        self.recortes: list[Recorte] = []  # onde a mão foi procurada
        self.modo = "so mao"  # "corpo" (duas etapas) ou "so mao" (ninguém no quadro: quadro inteiro)

    def __enter__(self) -> "DetectorGestos":
        return self

    def __exit__(self, *_) -> None:
        self._maos.close()
        self._pose.close()

    def processar(self, frame_bgr: np.ndarray, timestamp: float | None = None) -> Gesto:
        """Retorna o gesto estabilizado (troca só após `frames_confirmacao` frames iguais)."""
        t = time.monotonic() if timestamp is None else timestamp
        bruto = self._classificar(frame_bgr, t)
        if bruto == self._candidato:
            self._contagem += 1
        else:
            self._candidato, self._contagem = bruto, 1
        if self._contagem >= self._frames_confirmacao:
            self.gesto = bruto
        return self.gesto

    def _classificar(self, frame_bgr: np.ndarray, t: float) -> Gesto:
        altura, largura = frame_bgr.shape[:2]
        imagem = _mp_image(frame_bgr)

        # O modo VIDEO (Pose) exige timestamps estritamente crescentes.
        self._ultimo_ts_ms = max(int(t * 1000), self._ultimo_ts_ms + 1)
        pessoas = self._pose.detect_for_video(imagem, self._ultimo_ts_ms).pose_landmarks
        escala = np.array([largura, altura])
        self.corpos = [np.array([[lm.x, lm.y] for lm in p]) * escala for p in pessoas]
        self.recortes, self.maos = [], []

        if len(pessoas) > 1:
            self.modo = "corpo"
            return Gesto.VARIAS_PESSOAS

        recortes = None
        if len(pessoas) == 1:
            visibilidade = np.array([lm.visibility for lm in pessoas[0]])
            recortes = recortes_maos(self.corpos[0], visibilidade)
        if recortes is not None:
            self.modo = "corpo"
            self.recortes = recortes
            maos = [m for m in (self._mao_no_recorte(frame_bgr, r) for r in recortes) if m is not None]
            self.maos = remover_duplicadas(maos)
            # As mãos vieram dos recortes de um único corpo: dois punhos podem valer (DOIS_PUNHOS).
            return classificar_maos(self.maos, mesma_pessoa=True)  # nenhuma mão levantada: NENHUMA_MAO

        # Ninguém no quadro, ou só um pedaço da pessoa (sem ombros): provavelmente a mão está perto
        # da câmera e o corpo ficou de fora.
        self.modo = "so mao"
        resultado = self._maos.detect(imagem).hand_landmarks
        self.maos = [np.array([[lm.x, lm.y] for lm in mao]) * escala for mao in resultado]
        return classificar_maos(self.maos)

    def _mao_no_recorte(self, frame_bgr: np.ndarray, recorte: Recorte) -> np.ndarray | None:
        """Landmarks (21, 2), em pixels do quadro, da mão mais central do recorte (ou None)."""
        lado = int(round(recorte.lado))
        recorte = Recorte(recorte.cx, recorte.cy, float(lado))  # o mapeamento de volta usa o lado inteiro
        # getRectSubPix repete a borda quando o quadrado sai do quadro (mão perto da beirada).
        pedaco = cv2.getRectSubPix(frame_bgr, (lado, lado), (recorte.cx, recorte.cy))
        pedaco = cv2.resize(pedaco, (TAMANHO_RECORTE, TAMANHO_RECORTE), interpolation=cv2.INTER_LINEAR)
        resultado = self._maos.detect(_mp_image(pedaco)).hand_landmarks
        if not resultado:
            return None
        candidatas = [np.array([[lm.x, lm.y] for lm in mao]) for mao in resultado]
        # Se as duas mãos estão perto, o recorte de uma pode pegar a outra também: fica a do centro.
        central = min(candidatas, key=lambda m: float(np.linalg.norm(m[PALMA].mean(axis=0) - 0.5)))
        return para_quadro(central, recorte)


CONEXOES_MAO = [(c.start, c.end) for c in vision.HandLandmarksConnections.HAND_CONNECTIONS]
CONEXOES_CORPO = [(c.start, c.end) for c in vision.PoseLandmarksConnections.POSE_LANDMARKS]


def desenhar(frame: np.ndarray, detector: DetectorGestos) -> None:
    """Esqueleto do corpo (cinza), recortes (amarelo) e mãos (branco) do último frame."""
    for corpo in detector.corpos:
        pts = corpo.astype(int)
        for a, b in CONEXOES_CORPO:
            if a < 25 and b < 25:  # só tronco, braços e cabeça: as pernas só poluem a imagem
                cv2.line(frame, tuple(pts[a]), tuple(pts[b]), (160, 160, 160), 1)
    for r in detector.recortes:
        meio = r.lado / 2
        cv2.rectangle(frame, (int(r.cx - meio), int(r.cy - meio)), (int(r.cx + meio), int(r.cy + meio)),
                      (0, 255, 255), 1)
    for mao in detector.maos:
        pts = mao.astype(int)
        for a, b in CONEXOES_MAO:
            cv2.line(frame, tuple(pts[a]), tuple(pts[b]), (255, 255, 255), 2)


def executar_demo(indice_camera: int = 0) -> None:
    """Mostra a webcam com o gesto reconhecido. 'q' ou ESC para sair.

    Serve para testar as duas etapas sem drone: afaste-se da webcam e veja até onde a mão é achada.
    """
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
                desenhar(frame, detector)
                cv2.putText(frame, f"{gesto.value} [{detector.modo}]", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Gestos da mao", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        captura.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    executar_demo()

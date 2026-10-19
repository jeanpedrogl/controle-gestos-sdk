"""Máquina de estados da Fase 3 (Interação Humano-Enxame) da Flying Robots League.

Lógica pura, como `comando_voo.py` (sem ROS, sem câmera, sem MediaPipe): o nó chama `passo()` a
20 Hz com o que sabe agora (`Entrada`) e recebe de volta o que publicar (`Saida`). Assim cada regra
da competição vira uma transição testável em `test/test_missao_fase3.py`.

Fluxo (nenhuma tecla depois de iniciar: o regulamento proíbe tocar no computador na tentativa):

    ESPERANDO_DRONE --tudo pronto + contagem de CONTAGEM_S--> DECOLANDO -> PROCURANDO_OPERADOR
    -> APROXIMANDO -> GESTOS
    GESTOS --punho--> POUSANDO -> NA_BASE --punho--> DECOLANDO -> GESTOS -> ...
    dois punhos (depois de >= 1 base), em GESTOS ou na base -> VOLTANDO --punho--> POUSANDO -> FIM
    dois punhos em VOLTANDO -> GESTOS (volta cancelada)
    busca + aproximação além de BUSCA_MAX_S -> GESTOS mesmo sem enquadrar o operador
    telemetria perdida, drone desconectado, stick_mode desligado (em voo, na base ou decolando)
    ou decolagem sem pedido -> ABORTADO (não rearma)

A volta não é automática ao completar BASES_FASE pousos: a máquina conta pousos, não bases
diferentes (um pouso repetido também conta), então quem decide que acabou é o operador.

O terminal (que o juiz lê) mostra cada comando visual reconhecido, em todos os estados depois do
reconhecimento do operador, desde que o gesto se mantenha COMANDO_ESTAVEL_S, no formato
"COMANDO:<GESTO> -> <o que ele faz agora>" (o efeito depende do estado: o mesmo punho pousa em voo
e decola na base). As linhas são texto puro; `terminal.colorir` põe as cores na hora de imprimir.

A aproximação inicial é a única parte que usa a posição da pessoa para mover o drone, e só roda
uma vez: depois disso, nada segue o operador (rastrear o humano é proibido pela regra).

VOLTANDO ainda não é autônomo: os gestos levam o drone de volta e o punho pousa.

Câmera: pousado numa base (NA_BASE) o drone fica baixo e a câmera no horizonte só pega as pernas
do operador, então ela inclina GIMBAL_NA_BASE_GRAUS para cima; ao pedir a decolagem volta a
GIMBAL_NORMAL_GRAUS. Ângulos acima de 0° exigem `gimbal.set_pitch_range_extension` ligado (o nó liga
ao iniciar).

Convenção de eixos (REP 103): x = frente, y = esquerda, z = cima, yaw+ = anti-horário.
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from .comando_voo import DESCRICAO, PARADO, ZEROS, alvo_do_gesto, limitar

BASES_FASE = 6
CONTAGEM_S = 5  # contagem regressiva no terminal antes da primeira decolagem
CARENCIA_STICK_S = 3.0  # o celular leva um instante para ligar o stick mode
SUBIDA_S = 6.0  # a decolagem automática da DJI sobe ~1,2 m; só comanda depois disso
DECOLAGEM_MAX_S = 12.0  # sem `voando` nesse prazo: a decolagem não aconteceu
POUSO_MAX_S = 20.0  # ainda voando depois disso: o pouso não aconteceu
BUSCA_MAX_S = 90.0  # procura + aproximação, desde a primeira; uma volta a YAW_PROCURA leva ~42 s
APROXIMA_MAX_S = 30.0
PESSOA_PERDIDA_S = 1.0

YAW_PROCURA = 0.1  # anti-horário, ~8,5 °/s
GANHO_YAW = 0.15  # yaw = -GANHO_YAW * cx: pessoa à direita -> gira no sentido horário
AVANCO_MAX = 0.1  # 0,4 m/s
ALTURA_ALVO = 0.6  # fração do quadro ocupada pelo corpo quando "enquadrado" (ajustar em campo)
ALINHADO_PARA_AVANCAR = 0.3  # só avança com a pessoa perto do centro
CENTRADO = 0.15
ENQUADRADO_S = 0.5  # tempo enquadrado antes de passar o controle aos gestos
COMANDO_ESTAVEL_S = 0.3  # gesto mantido isso antes de aparecer no terminal (sem enxurrada de trocas)
SEM_COMANDO = ("NENHUMA_MAO", "DESCONHECIDO")  # não vão para o terminal

# Inclinação da câmera (gimbal pitch, graus; 0 = horizonte, positivo = para cima; Mini SE: -90..+20).
GIMBAL_NORMAL_GRAUS = 0.0
GIMBAL_NA_BASE_GRAUS = 15.0  # "um pouquinho para cima" (ajustar em campo; o máximo é +20)


class Estado(Enum):
    ESPERANDO_DRONE = auto()
    DECOLANDO = auto()
    PROCURANDO_OPERADOR = auto()
    APROXIMANDO = auto()
    GESTOS = auto()
    POUSANDO = auto()
    NA_BASE = auto()
    VOLTANDO = auto()
    FIM = auto()
    ABORTADO = auto()


EM_VOO_POR_GESTOS = (Estado.GESTOS, Estado.VOLTANDO)
TERMINAIS = (Estado.FIM, Estado.ABORTADO)
COM_OPERADOR = (Estado.GESTOS, Estado.VOLTANDO, Estado.POUSANDO, Estado.NA_BASE)  # gestos vão ao terminal


@dataclass(frozen=True)
class Pessoa:
    """Uma única pessoa na imagem da câmera do drone (o nó não passa nada com duas)."""

    cx: float  # centro horizontal na imagem SEM espelhar: -1 = esquerda do drone, +1 = direita
    altura: float  # altura aparente do corpo / altura do quadro, em [0, 1] (ver `pessoa_do_corpo`)


# Pontos do MediaPipe Pose usados para medir a pessoa.
OMBRO_ESQ, OMBRO_DIR, QUADRIL_ESQ, QUADRIL_DIR = 11, 12, 23, 24
# Altura do corpo estimada a partir de partes que continuam na imagem quando a pessoa está perto
# (as pernas saem primeiro): ~4,3 larguras de ombro e ~3,3 comprimentos de tronco (ombros-quadris).
FATOR_OMBROS = 4.3
FATOR_TRONCO = 3.3


def pessoa_do_corpo(pontos: np.ndarray, largura: int, altura: int) -> Pessoa:
    """`Pessoa` a partir dos 33 pontos (pixels) do Pose num quadro ESPELHADO (como o nó o processa).

    `altura` da Pessoa é a maior de três estimativas da altura aparente do corpo: a caixa dos pontos
    dentro da imagem, os ombros e o tronco. Assim, de perto (pernas fora do quadro) ela não encolhe,
    e de lado (ombros estreitos) o tronco e a caixa seguram. `cx` vem do meio dos ombros, com o
    espelho desfeito.
    """
    ys = np.clip(pontos[:, 1], 0, altura)
    caixa = float(ys.max() - ys.min())
    ombros = float(np.linalg.norm(pontos[OMBRO_ESQ] - pontos[OMBRO_DIR]))
    meio_ombros = (pontos[OMBRO_ESQ] + pontos[OMBRO_DIR]) / 2
    meio_quadris = (pontos[QUADRIL_ESQ] + pontos[QUADRIL_DIR]) / 2
    tronco = float(np.linalg.norm(meio_ombros - meio_quadris))
    estimada = max(caixa, FATOR_OMBROS * ombros, FATOR_TRONCO * tronco) / altura
    cx_espelhado = float(meio_ombros[0]) / largura * 2 - 1
    return Pessoa(cx=-cx_espelhado, altura=min(1.0, estimada))


@dataclass(frozen=True)
class Entrada:
    agora: float
    telemetria_ok: bool  # telemetria do celular chegou há pouco
    drone_conectado: bool = False
    voando: bool | None = None
    stick_mode: bool | None = None
    stick_assinado: bool = False  # o celular assina /drone/stick_mode
    gesto: str = PARADO  # `Gesto.name` estabilizado e recente (PARADO se velho)
    punho: bool = False  # evento: True só no passo em que o punho completa os 2 s (uma vez por punho)
    # O drone parou a ~0,3 m do chão e espera confirmação de pouso (`flight.landing_confirmation_needed`
    # de /phone/state): proteção de pouso da DJI, comum na borda de uma base suspensa.
    confirmar_pouso: bool = False
    voltar: bool = False  # evento: True só no passo em que os dois punhos (DOIS_PUNHOS) completam os 2 s
    pessoa: Pessoa | None = None


@dataclass
class Saida:
    velocidade: tuple[float, float, float, float] = ZEROS
    armado: bool = False  # stick_mode desejado; o nó publica só nas mudanças
    acao: str | None = None  # "decolar" | "pousar" | "cancelar_decolagem" (RPC)
    gimbal_pitch: float = GIMBAL_NORMAL_GRAUS  # ângulo desejado; o nó manda `gimbal.rotate` só nas mudanças
    mensagens: list[str] = field(default_factory=list)  # para o terminal (o juiz lê)


class MissaoFase3:
    def __init__(self, bases_fase: int = BASES_FASE, contagem_s: float = CONTAGEM_S) -> None:
        self.bases_fase = bases_fase
        self.contagem_s = contagem_s
        self._contagem_desde: float | None = None
        self._contagem_anunciada: int | None = None
        self.estado = Estado.ESPERANDO_DRONE
        self.bases_visitadas = 0
        self.armado = False
        self._desde = 0.0
        self._armado_em = 0.0
        self._pessoa_vista_em = -1e9
        self._enquadrado_desde: float | None = None
        self._pousando_final = False
        self._confirmacao_desde: float | None = None  # quando o drone pediu confirmação neste pouso
        self._busca_desde: float | None = None  # início da primeira procura pelo operador
        self._voltar_pedido = False  # dois punhos na base: a próxima decolagem vai para VOLTANDO
        self._gesto_candidato: str | None = None
        self._gesto_candidato_desde = 0.0
        self._gesto_impresso: tuple[str, str] | None = None  # (gesto, efeito) já no terminal
        self._msgs: list[str] = []

    # ---- utilitários -------------------------------------------------------

    def _ir(self, novo: Estado, agora: float, motivo: str = "") -> None:
        self._msgs.append(f"[{self.estado.name} -> {novo.name}] {motivo}".rstrip())
        self.estado, self._desde = novo, agora
        self._enquadrado_desde = None

    def _armar(self, agora: float) -> None:
        if not self.armado:
            self.armado, self._armado_em = True, agora

    def _desarmar(self) -> None:
        self.armado = False

    def _abortar(self, agora: float, motivo: str) -> None:
        self._desarmar()
        self._ir(Estado.ABORTADO, agora, motivo)

    def _motivo_aborto(self, e: Entrada) -> str | None:
        if self.estado is Estado.ESPERANDO_DRONE:
            return None  # sem telemetria ainda é normal aqui
        if not e.telemetria_ok:
            return "perdi a telemetria do celular"
        if not e.drone_conectado:
            return "o celular nao enxerga o drone"
        if (self.armado and e.voando and e.stick_mode is False
                and e.agora - self._armado_em > CARENCIA_STICK_S):
            return "o celular retomou o controle (panico, RC ou app em segundo plano)"
        return None

    # ---- passo (20 Hz) -----------------------------------------------------

    def passo(self, e: Entrada) -> Saida:
        self._msgs = []
        if self.estado not in TERMINAIS:
            motivo = self._motivo_aborto(e)
            if motivo:
                self._abortar(e.agora, motivo)
        self._anunciar_gesto(e)

        tratar = {
            Estado.ESPERANDO_DRONE: self._esperando,
            Estado.DECOLANDO: self._decolando,
            Estado.PROCURANDO_OPERADOR: self._procurando,
            Estado.APROXIMANDO: self._aproximando,
            Estado.GESTOS: self._por_gestos,
            Estado.VOLTANDO: self._por_gestos,
            Estado.POUSANDO: self._pousando,
            Estado.NA_BASE: self._na_base,
        }.get(self.estado)
        velocidade, acao = tratar(e) if tratar else (ZEROS, None)
        velocidade = tuple(limitar(v) for v in velocidade)
        # Decidido depois do estado: o punho que decola da base já devolve a câmera ao normal.
        gimbal = GIMBAL_NA_BASE_GRAUS if self.estado is Estado.NA_BASE else GIMBAL_NORMAL_GRAUS
        return Saida(velocidade=velocidade, armado=self.armado, acao=acao, gimbal_pitch=gimbal,
                     mensagens=self._msgs)

    def _anunciar_gesto(self, e: Entrada) -> None:
        """Põe no terminal cada comando visual que se manteve COMANDO_ESTAVEL_S (regra da Fase 3)."""
        if self.estado not in COM_OPERADOR:
            self._gesto_candidato = self._gesto_impresso = None
            return
        if e.gesto != self._gesto_candidato:
            self._gesto_candidato, self._gesto_candidato_desde = e.gesto, e.agora
        elif e.agora - self._gesto_candidato_desde >= COMANDO_ESTAVEL_S:
            # Compara o par: o mesmo gesto mantido muda de efeito (punho: "pousar aqui" vira
            # "confirmar o pouso" quando o drone pede). "Sem comando" também conta, então repetir um
            # gesto depois de soltar imprime de novo.
            atual = (e.gesto, self._efeito(e.gesto, e))
            if atual != self._gesto_impresso:
                self._gesto_impresso = atual
                if e.gesto not in SEM_COMANDO:
                    self._msgs.append(f"COMANDO:{atual[0]} -> {atual[1]}")

    def _efeito(self, gesto: str, e: Entrada) -> str:
        """O que `gesto` faz no estado atual (texto do terminal; tem que bater com o que a máquina faz)."""
        segure = " (segure 2 s)"
        if gesto == "PUNHO_FECHADO":
            if self.estado is Estado.GESTOS:
                return "pousar aqui" + segure
            if self.estado is Estado.VOLTANDO:
                return "pousar na base de decolagem" + segure
            if self.estado is Estado.NA_BASE:
                return "decolar para a proxima base" + segure
            if e.confirmar_pouso:
                return "confirmar o pouso" + segure
            return "nenhum (o drone ja esta pousando)"
        if gesto == "DOIS_PUNHOS":
            if self.estado is Estado.VOLTANDO:
                return "cancelar a volta" + segure
            if self.estado is Estado.POUSANDO:
                return "nenhum (o drone esta pousando)"
            if self.bases_visitadas == 0:
                return "nenhum (nenhuma base visitada ainda)"
            if self.estado is Estado.NA_BASE:
                return "decolar e voltar para a base de decolagem" + segure
            return "voltar para a base de decolagem" + segure
        if self.estado is Estado.POUSANDO:
            return "nenhum (o drone esta pousando)"
        if self.estado is Estado.NA_BASE:
            return "nenhum (o drone esta na base)"
        if gesto == "VARIAS_MAOS":
            return "pairar (mais de uma mao na imagem)"
        if gesto == "VARIAS_PESSOAS":
            return "pairar (mais de uma pessoa na imagem)"
        return DESCRICAO.get(gesto, "pairar (gesto sem comando)")

    @staticmethod
    def _onde(p: Pessoa) -> str:
        return f"centro x={p.cx:+.2f} (-1 esquerda, +1 direita), altura {p.altura:.0%} da imagem"

    def _passou_da_busca(self, e: Entrada) -> bool:
        if self._busca_desde is not None and e.agora - self._busca_desde > BUSCA_MAX_S:
            self._ir(Estado.GESTOS, e.agora, "operador nao enquadrado a tempo: controle por gestos assim mesmo")
            return True
        return False

    # ---- estados -----------------------------------------------------------

    def _esperando(self, e: Entrada):
        """Espera tudo pronto e conta `contagem_s` no terminal; se algo cair no meio, recomeça do zero."""
        falta_o_que = self._falta_para_decolar(e)
        if falta_o_que:
            if self._contagem_desde is not None:
                self._msgs.append(f"Contagem cancelada: {falta_o_que}")
                self._contagem_desde = None
            return ZEROS, None
        if self._contagem_desde is None:
            self._contagem_desde, self._contagem_anunciada = e.agora, None
        falta = self.contagem_s - (e.agora - self._contagem_desde)
        if falta > 0:
            segundos = math.ceil(falta)
            if segundos != self._contagem_anunciada:
                self._contagem_anunciada = segundos
                self._msgs.append(f"Decolando em {segundos}...")
            return ZEROS, None
        self._armar(e.agora)
        self._ir(Estado.DECOLANDO, e.agora, "drone pronto: decolando")
        return ZEROS, "decolar"

    @staticmethod
    def _falta_para_decolar(e: Entrada) -> str | None:
        if not e.telemetria_ok:
            return "sem telemetria do celular"
        if not e.drone_conectado:
            return "o celular nao enxerga o drone"
        if not e.stick_assinado:
            return "o celular nao assinou /drone/stick_mode"
        if e.voando is not False:
            return "o drone nao esta confirmado no chao"
        return None

    def _decolando(self, e: Entrada):
        if (not e.voando and e.stick_mode is False
                and e.agora - self._armado_em > CARENCIA_STICK_S):
            # Pânico/ASSUMIR CONTROLE entre o pedido e a subida: a regra geral só vale em voo.
            self._abortar(e.agora, "o celular desligou o stick mode antes de subir: decolagem cancelada")
            return ZEROS, "cancelar_decolagem"
        if e.voando and e.agora - self._desde >= SUBIDA_S:
            if self.bases_visitadas == 0:
                self._busca_desde = e.agora
                self._ir(Estado.PROCURANDO_OPERADOR, e.agora, "no ar: procurando o operador")
            elif self._voltar_pedido:
                self._voltar_pedido = False
                self._ir(Estado.VOLTANDO, e.agora, "guie o drone ate a base de decolagem e feche o punho para pousar")
            else:
                self._ir(Estado.GESTOS, e.agora, "no ar: controle por gestos")
        elif not e.voando and e.agora - self._desde > DECOLAGEM_MAX_S:
            if self.bases_visitadas == 0:
                self._abortar(e.agora, "a decolagem nao aconteceu")
            else:
                self._ir(Estado.NA_BASE, e.agora, "a decolagem nao aconteceu")
        return ZEROS, None

    def _procurando(self, e: Entrada):
        if e.pessoa is not None:
            self._pessoa_vista_em = e.agora
            self._msgs.append(f"OPERADOR DETECTADO: {self._onde(e.pessoa)}")
            self._ir(Estado.APROXIMANDO, e.agora, "aproximando do operador")
            return ZEROS, None
        if self._passou_da_busca(e):
            return ZEROS, None
        return (0.0, 0.0, 0.0, YAW_PROCURA), None

    def _aproximando(self, e: Entrada):
        if self._passou_da_busca(e):
            return ZEROS, None
        if e.pessoa is None:
            if e.agora - self._pessoa_vista_em > PESSOA_PERDIDA_S:
                self._ir(Estado.PROCURANDO_OPERADOR, e.agora, "perdi o operador")
            return ZEROS, None
        self._pessoa_vista_em = e.agora
        p = e.pessoa
        if e.agora - self._desde > APROXIMA_MAX_S:
            self._ir(Estado.GESTOS, e.agora, "aproximacao incompleta: controle por gestos")
            return ZEROS, None
        if abs(p.cx) < CENTRADO and p.altura >= ALTURA_ALVO:
            if self._enquadrado_desde is None:
                self._enquadrado_desde = e.agora
            elif e.agora - self._enquadrado_desde >= ENQUADRADO_S:
                self._msgs.append(f"OPERADOR RECONHECIDO: {self._onde(p)}")
                self._ir(Estado.GESTOS, e.agora, "controle por gestos")
            return ZEROS, None
        self._enquadrado_desde = None
        yaw = -GANHO_YAW * p.cx
        frente = 0.0
        if abs(p.cx) < ALINHADO_PARA_AVANCAR and p.altura < ALTURA_ALVO:
            frente = AVANCO_MAX * min(1.0, (ALTURA_ALVO - p.altura) / 0.2)
        return (frente, 0.0, 0.0, yaw), None

    def _por_gestos(self, e: Entrada):
        if e.voltar:
            if self.estado is Estado.VOLTANDO:
                self._ir(Estado.GESTOS, e.agora, "volta cancelada: controle por gestos")
            elif self.bases_visitadas == 0:
                self._msgs.append("Dois punhos ignorados: nenhuma base visitada ainda")
            else:
                self._ir(Estado.VOLTANDO, e.agora, "guie o drone ate a base de decolagem e feche o punho para pousar")
            return ZEROS, None
        if e.punho:
            self._pousando_final = self.estado is Estado.VOLTANDO
            alvo = "base de decolagem" if self._pousando_final else f"base {self.bases_visitadas + 1}"
            self._ir(Estado.POUSANDO, e.agora, f"pousando na {alvo}")
            return ZEROS, "pousar"
        return alvo_do_gesto(e.gesto), None

    def _pousando(self, e: Entrada):
        """Espera tocar o chão. Se o drone parar pedindo confirmação, o operador confirma com o punho.

        Os gestos de movimento continuam desligados enquanto isso (o drone está a ~0,3 m da base).
        O prazo POUSO_MAX_S recomeça quando o pedido aparece; esgotado sem confirmação, o controle
        volta aos gestos para reposicionar o drone (e um novo punho pousa de novo).
        """
        if e.voando is False:
            self._confirmacao_desde = None
            if self._pousando_final:
                self._desarmar()
                self._ir(Estado.FIM, e.agora, f"pousou na base de decolagem ({self.bases_visitadas} bases)")
            else:
                self.bases_visitadas += 1
                self._ir(Estado.NA_BASE, e.agora, f"POUSO {self.bases_visitadas}/{self.bases_fase}")
                if self.bases_visitadas >= self.bases_fase:
                    self._msgs.append("Todas as bases: dois punhos para voltar a base de decolagem")
            return ZEROS, None
        acao = None
        if e.confirmar_pouso:
            if self._confirmacao_desde is None:
                self._confirmacao_desde = e.agora
                self._msgs.append("DRONE PEDE CONFIRMACAO DE POUSO: feche o punho para pousar")
            if e.punho:
                self._msgs.append("POUSO CONFIRMADO pelo operador")
                acao = "pousar"  # o app transforma em confirmLanding enquanto o drone espera
        inicio = self._confirmacao_desde if self._confirmacao_desde is not None else self._desde
        if e.agora - inicio > POUSO_MAX_S:
            self._confirmacao_desde = None
            volta = Estado.VOLTANDO if self._pousando_final else Estado.GESTOS
            self._ir(volta, e.agora, "o pouso nao terminou: reposicione e feche o punho de novo")
        return ZEROS, acao

    def _na_base(self, e: Entrada):
        """Pousado numa base, motores parados: o punho decola para a próxima base; dois punhos, para voltar.

        O `stick_mode` da telemetria é o do app, que nunca o desliga num pouso: no chão, `False`
        quer dizer que alguém apertou PÂNICO ou ASSUMIR CONTROLE, ou que o app saiu da tela. Isso é
        respeitado como em voo: não decola mais.
        """
        if e.voando:
            self._abortar(e.agora, "o drone decolou sem pedido (RC?)")
            return ZEROS, None
        if e.stick_mode is False:
            self._abortar(e.agora, "o celular desligou o stick mode com o drone na base")
            return ZEROS, None
        if (e.punho or e.voltar) and e.voando is False:
            self._voltar_pedido = e.voltar
            destino = "para voltar a base de decolagem" if e.voltar else "para a proxima base"
            self._armar(e.agora)
            self._ir(Estado.DECOLANDO, e.agora, f"decolando da base {self.bases_visitadas} {destino}")
            return ZEROS, "decolar"
        return ZEROS, None

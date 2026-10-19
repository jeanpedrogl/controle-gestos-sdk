# controle-visao

Visão computacional (reconhecimento de gestos de mão) rodando no computador,
usada para pilotar um drone **DJI Mini SE** cujo app de celular (**Drone Bridge**, repositório `drone-bridge`)
atua apenas como ponte entre este computador e o controle físico da DJI.

```
Câmera do drone → celular (JPEG) → rosbridge (WebSocket) → integracao_visao_drone.py + visao_mao.py
                                                                              │ comandos de voo (ROS2)
                                                                              │
                                                                     Wi-Fi (hotspot do PC)
                                                                              │
                                                                     Drone Bridge (celular)
                                                                              │
                                                              DJI Mobile SDK v4 → controle RC → drone
```

A **imagem usada para reconhecer os gestos é a da câmera do próprio drone**: o celular recebe o
vídeo via DJI SDK, comprime em JPEG (padrão 8 fps, 640 px) e publica em
`/phone/drone_camera/compressed`; o `piloto_gestos` classifica a mão em cada quadro e mostra a
mesma janela, com o esqueleto da mão, o gesto e o estado. Ou seja, o drone precisa estar
apontado para quem faz os gestos, e a webcam do computador não é usada.

## Por que ROS2 + rosbridge, e não ROS2 "puro" até o celular

Não existe cliente ROS2 nativo para Android (`rclpy`/`rclcpp` não rodam lá).
O padrão usado pela própria comunidade ROS2 para clientes móveis/web é o
[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite): ele expõe
tópicos ROS2 reais via WebSocket/JSON. O app Android implementa um cliente
WebSocket simples (`RosBridgeClient.kt`) falando esse protocolo — nenhuma
dependência especial no celular, e qualquer tópico/serviço ROS2 novo que você
criar aqui já fica acessível por esse mesmo canal.

## Pré-requisitos

- Ubuntu 22.04 (jammy) — é o que já está instalado nesta máquina.
- ROS2 **Humble** (combina com o Ubuntu 22.04); o `setup_ambiente.sh` instala se faltar.

## Setup (uma vez)

```bash
./setup_ambiente.sh
```

Isso instala o ROS2 Humble + `ros-humble-rosbridge-suite` via apt (pede sudo)
e cria um venv Python (`.venv`, com `--system-site-packages` para enxergar o
`rclpy` instalado pelo apt) com `mediapipe`, `opencv-python` e `numpy`.

## Compilar e rodar

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 -m colcon build --symlink-install
source install/setup.bash
ros2 launch ponte_visao_drone sistema.launch.py              # modo teclado (treino, padrão)
ros2 launch ponte_visao_drone sistema.launch.py modo:=fase3  # Fase 3 da Flying Robots League
```

No **modo Fase 3** quem decide é a máquina de estados (`missao_fase3.py`) e nenhuma tecla funciona,
só `q`/Esc para sair e desarmar (o regulamento proíbe tocar no computador durante a tentativa).
Assim que celular e drone estão conectados, o terminal faz uma contagem de 5 s e o drone decola
sozinho, procura o operador, se aproxima uma vez para enquadrá-lo e passa o controle aos gestos: o
punho fechado por 2 s pousa (em voo) ou decola (numa base), os dois punhos por 2 s voltam para a
base de decolagem. O terminal mostra o reconhecimento do operador e cada comando visual, em cores
(`COMANDO:GESTO -> o que ele faz`), e a mesma coisa vai, sem cores, para
`~/.ros/log/fase3-<data>-<hora>.txt` (o log das atividades que o regulamento pede).

Use `python3 -m colcon build`, não só `colcon build`: o executável `colcon` do apt tem seu
próprio shebang fixo pro Python do sistema, então chamado direto ele ignora o venv ativado e
gera os nós apontando pro Python errado (sem o `mediapipe` instalado).

Isso sobe duas coisas: o `rosbridge_websocket` (porta `9090`, padrão) e o nó
`piloto_gestos` (assina o vídeo do drone e mostra uma janela com a imagem, o gesto detectado e o
estado; fica em "Aguardando video do drone" até o celular começar a publicar). O nó
`visor_camera_drone` (só monitoramento) não sobe mais junto, porque o piloto já mostra o vídeo;
se quiser, `ros2 run ponte_visao_drone visor_camera_drone`.
Tudo de uma vez, com compilação automática: `./iniciar.sh`.

Para rodar só a detecção, sem ROS2 nem drone (debug rápido, com a webcam do computador):

```bash
.venv/bin/python -m ponte_visao_drone.visao_mao
```

## Rede: computador como hotspot

O plano é o **computador criar o ponto de acesso Wi-Fi** e o celular se
conectar nele (assim os dois ficam na mesma rede sem depender de um roteador
externo). No GNOME (Configurações → Wi-Fi → menu "⋮" → "Criar rede Wi-Fi..."),
isso tipicamente deixa o computador em `10.42.0.1` e distribui IPs do tipo
`10.42.0.x` para quem conectar — mas confirme o IP real com `ip addr` depois
de ligar o hotspot, pois pode variar. **No app do celular**, configure esse IP
nas configurações de conexão (ponte ROS2) antes de conectar; a porta padrão do
rosbridge é `9090`.

## Controles (janela "Piloto por gestos", teclado em foco)

| Tecla | Ação |
|---|---|
| `a` | Arma/desarma o envio de comandos (o drone só se move com "armado"). **Só arma com o celular conectado** (telemetria chegando, drone visível e celular assinando `/drone/stick_mode`); senão a janela explica por quê. |
| `t` | Decolar: aperte **duas vezes** em até 3 s (também: punho fechado por 2 s, com o drone no chão **e o programa armado**). Não decola sem telemetria, sem drone ou com o drone já no ar. |
| `l` | Pousar (também: punho fechado por 2 s, com o drone no ar) |
| `1` `2` `3` `4` | Qualidade do vídeo do drone (`video.set_quality`), da mais leve à mais nítida: **1** 15 fps/320 px/q40, **2** 12 fps/480 px/q50, **3** 8 fps/640 px/q50 (padrão do celular), **4** 8 fps/960 px/q70. Menos pixels = menos atraso, mas a mão fica pequena; mais pixels = detecção melhor, mas mais atraso. A janela mostra o que o celular confirmou. Edite `PRESETS_VIDEO` em `integracao_visao_drone.py` para ajustar. |
| `q` / `Esc` | Sai (desarma antes) |

A janela mostra o gesto, se está armado e uma linha de estado do celular (drone, bateria, voando,
stick mode), além do resultado de decolar/pousar (`flight.take_off`/`flight.land` por RPC).

## Mapeamento gesto → comando

| Gesto (a pose é mantida, não há gestos de arrasto) | Comando |
|---|---|
| Indicador para cima (demais dedos fechados) | Subir |
| Indicador para baixo (demais dedos fechados) | Descer |
| Punho fechado com o polegar para a direita / esquerda | Deslocar para a **sua** direita / esquerda (o drone está de frente para você, então para ele é o lado oposto: polegar para a direita = esquerda do drone, `y+`) |
| "V" (indicador e médio) | Girar (yaw) sempre no sentido anti-horário (para a esquerda). Não há giro horário por gesto: como a câmera gira junto, o lado da imagem em que a mão está não serve para escolher o sentido. |
| Hang loose (polegar e mínimo esticados) | Frente (na direção de quem faz o gesto), a **metade** da velocidade dos outros eixos |
| Mão aberta, palma para a câmera, dedos para cima | Trás |
| Punho fechado (polegar recolhido) por ~2 s | **Decolar** se o drone está no chão, **pousar** se está voando (pela telemetria `voando`; sem telemetria não faz nada). Decolar por gesto só funciona **armado** (`a`); desarmado, o punho no chão só avisa na janela. Pousar vale sempre. A janela mostra a contagem; falhas de detecção de até 0,4 s no meio não zeram a contagem, mudar de gesto por mais tempo que isso cancela, e só age de novo depois de soltar o punho. Vale o polegar por cima dos dedos ou ao lado do indicador; polegar saindo da mão (para cima, para baixo, para o lado) não é mão fechada. Uma decolagem já pedida não é repetida por 10 s (uma recusada pelo celular libera nova tentativa). |
| Nenhuma mão / qualquer outra pose | Pairar |
| **Mais de uma mão levantada** ou **mais de uma pessoa** na imagem | Pairar: sem comando nenhum, e a contagem do punho cai. Evita que alguém que entre no quadro do drone assuma o controle. |
| **Os dois punhos fechados** (as duas mãos levantadas, da mesma pessoa) | Pairar no modo atual. No modo Fase 3 (`modo:=fase3`) é o "voltar para a base de decolagem". Só vale quando as duas mãos vieram do corpo de **uma** pessoa detectada; sem o corpo (mão perto da câmera), duas mãos continuam sendo "mais de uma mão". |

Direita/esquerda são as do ponto de vista do usuário (o quadro do drone é espelhado, como num espelho). O comando
vale enquanto o gesto é mantido; sem gesto reconhecido o drone paira. Polegar para cima/para baixo
ou torto **não** é comando (e não conta como punho, então não pousa).

Os valores são **20 %** do limite do app (`VELOCIDADE_*` em `comando_voo.py`: 1,0 = 4 m/s na
horizontal, 2 m/s na vertical, 85 °/s no giro), com rampa de ~0,3 s subindo e ~0,15 s descendo. Exceção: "frente" (hang loose) usa metade (`FATOR_FRENTE`, 10 % do limite = 0,4 m/s).
Os limiares das poses (`FATOR_*` e `LIMIAR_*` em `poses_mao.py`) foram escolhidos sem testar com uma
mão real; confira com `.venv/bin/python -m ponte_visao_drone.visao_mao` e ajuste se preciso.

## Como a mão é encontrada (duas etapas)

Na câmera grande-angular do drone a mão ocupa poucos pixels: o detector de palma do MediaPipe, que
olha o quadro inteiro reduzido para ~192 px, só a achava com ela a uns 20 cm. Por isso a detecção é
em duas etapas ("zoom por software", `visao_mao.py` + `recorte_mao.py`):

1. O **MediaPipe Pose** acha a *pessoa* no quadro inteiro (o corpo é grande e fácil de achar a
   alguns metros) e dá os pontos dos ombros, pulsos e mãos.
2. Para cada mão **levantada** (pulso acima do meio do tronco), um quadrado do quadro original em
   volta da mão (1,1 largura de ombro de lado) é recortado, e o modelo de mão roda só nele. Os
   gestos são os mesmos da tabela acima.

Consequências práticas:
- **Faça os gestos com a mão acima da cintura.** Mão abaixada (braço solto, que costuma parecer
  um punho) não é procurada e não comanda nada.
- **Mais de uma pessoa no quadro**: nenhum comando (`VARIAS_PESSOAS`), o drone paira.
- **Sem pessoa no quadro**, ou com os ombros fora da imagem (mão bem perto da câmera), volta ao
  modo antigo: mão procurada no quadro inteiro. A janela mostra o modo (`corpo` ou `so mao`), o
  esqueleto do corpo em cinza e os recortes em amarelo.
- **A resolução do vídeo agora importa**: o recorte só tem detalhe se o quadro tiver pixels. Prefira
  as teclas `3`/`4` (ou um preset com mais pixels e menos fps).
- Custo nesta máquina (Ryzen 5 2500U, só CPU): ~95 ms por quadro com uma mão levantada, ~140 ms com
  as duas; o vídeo chega a ~8 fps, então cabe, mas sobra pouco.
- Para testar sem drone: `cd src/ponte_visao_drone && ../../.venv/bin/python -m ponte_visao_drone.visao_mao`
  usa a webcam com as mesmas duas etapas; afaste-se e veja até onde a mão é achada.
- O modelo `pose_landmarker_full.task` fica em `ponte_visao_drone/modelos/` (baixado sozinho se faltar).

## Tópicos ROS2 (ponte com o celular)

Publicados por este pacote (consumidos pelo celular):
- `/drone/cmd_vel` (`geometry_msgs/Twist`) — velocidade, a **20 Hz fixos** (mesmo em zeros) enquanto armado; se os gestos param de chegar (vídeo do drone travado ou atrasado, mais de 0,6 s sem quadro novo), manda zeros.
- `/drone/stick_mode` (`std_msgs/Bool`) — só quando o estado muda (quando `false`, o controle físico da DJI assume).
- `/drone/rpc/request` (`std_msgs/String`, JSON) — `flight.take_off`, `flight.land` e `video.set_quality`, com resposta em `/phone/rpc/response` (ids únicos por programa; a resposta aparece na janela). Também `state.stream_start` (seção `flight`, 250 ms), pedido de novo sempre que `/phone/state` some.

Publicados pelo celular (consumidos aqui):
- `/phone/drone_camera/compressed` (`sensor_msgs/CompressedImage`) — vídeo do drone, ~8fps JPEG; **é a entrada da visão**. Dá para pedir mais fps/resolução ao celular com o RPC `video.set_quality`.
- `/phone/telemetria` (`std_msgs/String`, JSON, 2 Hz) — `drone_conectado`, `stick_mode`, `bateria_percent`, `altitude_m`, `velocidade_ms`, `satelites`, `voando`. Usada para só armar com o celular ouvindo e para desarmar se o celular retomar o controle.
- `/phone/rpc/response` (`std_msgs/String`, JSON) — respostas dos comandos RPC.
- `/phone/state` (`std_msgs/String`, JSON) — estado do voo; só `flight.landing_confirmation_needed` é usado: o drone parou a ~0,3 m do chão esperando confirmação de pouso (a janela avisa; punho fechado ou `l` confirma).

O contrato completo (todos os tópicos e comandos RPC) está no `PROTOCOLO.md` e no `MANUAL_DESENVOLVEDOR.md` do repositório `drone-bridge`.

Para estender (novos algoritmos/comandos no futuro): basta criar um novo
publisher/subscriber em qualquer um dos lados — o rosbridge já expõe
automaticamente qualquer tópico novo que apareça no grafo ROS2, sem precisar
tocar na ponte em si.

## Segurança

- O botão de pânico do app do celular funciona **sempre localmente**, mesmo
  sem conexão com este computador — não depende do ROS2.
- Se `/drone/cmd_vel` parar de chegar por 500ms (Wi-Fi caiu, este programa
  travou), o celular para de mandar velocidade e o drone paira **ainda em stick mode**;
  o controle físico só volta com "ASSUMIR CONTROLE (RC)" ou pânico no celular.
- Este programa **desarma** (Twist zerado e depois `stick_mode: false`) ao sair por `q`/`Esc`,
  `Ctrl+C`, `kill` ou erro. Para isso o nó inicia o ROS com
  `SignalHandlerOptions.NO` (no Humble o `Ctrl+C` padrão invalida o contexto antes do `finally`).
- Se a telemetria disser `stick_mode: false` (pânico, "assumir controle", app em segundo plano) ou
  sumir, o programa **desarma e não rearma sozinho**.
- Teste sem risco num domínio ROS isolado: `export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`. No domínio
  padrão, com o celular conectado, `t`/`l` e o gesto de punho atingem o drone de verdade.
- A rede do rosbridge não tem autenticação nem criptografia: use um hotspot com senha.

## Testes

A lógica pura está em `comando_voo.py` (rampa, mapeamento gesto → velocidade) e `poses_mao.py`
(classificação das poses a partir dos landmarks) e é testada sem ROS, MediaPipe nem câmera:

```bash
cd src/ponte_visao_drone && python3 -m pytest test
```

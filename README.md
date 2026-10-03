# controle-visao

Visão computacional (reconhecimento de gestos de mão) rodando no computador,
usada para pilotar um drone **DJI Mini SE** cujo app de celular (**Drone Bridge**, repositório `drone-bridge`)
atua apenas como ponte entre este computador e o controle físico da DJI.

```
Webcam (computador) → visao_mao.py → integracao_visao_drone.py → ROS2 → rosbridge (WebSocket)
                                                                              │
                                                                     Wi-Fi (hotspot do PC)
                                                                              │
                                                                     Drone Bridge (celular)
                                                                              │
                                                              DJI Mobile SDK v4 → controle RC → drone
```

O vídeo da câmera do drone faz o caminho inverso: o celular recebe via DJI SDK,
comprime em JPEG e publica em ROS2; `visor_camera_drone.py` mostra essa janela
no computador. Essa câmera é só para acompanhar o voo — quem controla o drone
é sempre a webcam do computador.

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
ros2 launch ponte_visao_drone sistema.launch.py
```

Use `python3 -m colcon build`, não só `colcon build`: o executável `colcon` do apt tem seu
próprio shebang fixo pro Python do sistema, então chamado direto ele ignora o venv ativado e
gera os nós apontando pro Python errado (sem o `mediapipe` instalado).

Isso sobe três coisas: o `rosbridge_websocket` (porta `9090`, padrão), o nó
`piloto_gestos` (abre a webcam, mostra uma janela com o gesto detectado) e o
nó `visor_camera_drone` (mostra o vídeo do drone assim que o celular começar
a publicar).

Outra webcam: `ros2 launch ponte_visao_drone sistema.launch.py camera_index:=1`.
Tudo de uma vez, com compilação automática: `./iniciar.sh`.

Para rodar só a detecção, sem ROS2 nem drone (debug rápido da webcam):

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
| `t` | Decolar: aperte **duas vezes** em até 3 s (também: polegar para cima por 2 s). Não decola sem telemetria, sem drone ou com o drone já no ar. |
| `l` | Pousar (também: polegar para baixo por 2 s) |
| `q` / `Esc` | Sai (desarma antes) |

A janela mostra o gesto, se está armado e uma linha de estado do celular (drone, bateria, voando,
stick mode), além do resultado de decolar/pousar (`flight.take_off`/`flight.land` por RPC).

## Mapeamento gesto → comando

| Gesto | Comando |
|---|---|
| Nenhuma mão / Punho fechado / Mão aberta parada / Desconhecido | Pairar |
| Polegar para cima (2s) | Decolar |
| Polegar para baixo (2s) | Pousar |
| V esquerda / V direita | Girar (yaw) esquerda / direita |
| Mão subindo / descendo | Subir / descer |
| Mão esquerda / direita | Deslocar lateralmente |
| Mão se aproximando (palma cresce) | Drone vai para frente |
| Mão se afastando (palma encolhe) | Drone vai para trás |

Os valores são **20 %** do limite do app (`VELOCIDADE_*` em `comando_voo.py`: 1,0 = 4 m/s na
horizontal, 2 m/s na vertical, 85 °/s no giro), com rampa de ~0,3 s subindo e ~0,15 s descendo. Depois de
um movimento, o gesto **oposto** nos 0,8 s seguintes é tratado como "a mão voltando ao centro" e vira
pairar (a mão sobe, volta descendo, e o drone não desce).

## Tópicos ROS2 (ponte com o celular)

Publicados por este pacote (consumidos pelo celular):
- `/drone/cmd_vel` (`geometry_msgs/Twist`) — velocidade, a **20 Hz fixos** (mesmo em zeros) enquanto armado; se os gestos param de chegar (webcam travada), manda zeros.
- `/drone/stick_mode` (`std_msgs/Bool`) — só quando o estado muda (quando `false`, o controle físico da DJI assume).
- `/drone/rpc/request` (`std_msgs/String`, JSON) — `flight.take_off` e `flight.land`, com resposta em `/phone/rpc/response` (ids únicos por programa; a resposta aparece na janela).

Publicados pelo celular (consumidos aqui):
- `/phone/drone_camera/compressed` (`sensor_msgs/CompressedImage`) — vídeo do drone, ~8fps JPEG.
- `/phone/telemetria` (`std_msgs/String`, JSON, 2 Hz) — `drone_conectado`, `stick_mode`, `bateria_percent`, `altitude_m`, `velocidade_ms`, `satelites`, `voando`. Usada para só armar com o celular ouvindo e para desarmar se o celular retomar o controle.
- `/phone/rpc/response` (`std_msgs/String`, JSON) — respostas dos comandos RPC.

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
  `Ctrl+C`, `kill`, erro ou webcam perdida. Para isso o nó inicia o ROS com
  `SignalHandlerOptions.NO` (no Humble o `Ctrl+C` padrão invalida o contexto antes do `finally`).
- Se a telemetria disser `stick_mode: false` (pânico, "assumir controle", app em segundo plano) ou
  sumir, o programa **desarma e não rearma sozinho**.
- Teste sem risco num domínio ROS isolado: `export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`. No domínio
  padrão, com o celular conectado, `t`/`l` e o gesto de polegar atingem o drone de verdade.
- A rede do rosbridge não tem autenticação nem criptografia: use um hotspot com senha.

## Testes

A lógica pura (rampa, mapeamento, filtro de retorno) está em `comando_voo.py` e é testada sem ROS
nem webcam:

```bash
cd src/ponte_visao_drone && python3 -m pytest test/test_comando_voo.py
```

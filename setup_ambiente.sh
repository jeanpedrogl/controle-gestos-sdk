#!/usr/bin/env bash
# Instala o ROS2 Humble + rosbridge-suite (se ainda não estiverem instalados) e
# cria um venv Python com acesso aos pacotes do sistema (necessário para o
# rclpy, que é instalado via apt, não pip) com as dependências de visão.
#
# Rode uma vez, manualmente (precisa de sudo para a parte do ROS2):
#   ./setup_ambiente.sh
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v ros2 &>/dev/null; then
    echo "== Instalando ROS2 Humble (requer Ubuntu 22.04 'jammy') =="
    sudo apt update
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop ros-humble-rosbridge-suite \
        python3-colcon-common-extensions python3-rosdep
    sudo rosdep init 2>/dev/null || true
    rosdep update
else
    echo "== ROS2 já instalado ($(ros2 --version 2>&1 || true)), pulando =="
    if ! dpkg -s ros-humble-rosbridge-suite &>/dev/null; then
        echo "== Instalando ros-humble-rosbridge-suite =="
        sudo apt update && sudo apt install -y ros-humble-rosbridge-suite
    fi
fi

echo "== Criando venv Python (--system-site-packages, para enxergar o rclpy) =="
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt

# pip só pula um pacote já "satisfeito" em QUALQUER lugar do sys.path — como o venv é
# --system-site-packages, isso inclui o python3-matplotlib do apt (compilado contra numpy
# 1.x). Sem isso, o import do mediapipe (que usa matplotlib internamente) quebra assim que o
# numpy 2.x do venv entra em cena. Forçar uma cópia nova aqui faz o venv priorizar a sua
# (o site-packages do venv vem antes do do sistema no sys.path).
echo "== Garantindo matplotlib compatível com numpy 2.x dentro do venv =="
.venv/bin/pip install --force-reinstall --no-warn-script-location matplotlib

cat <<'EOF'

Pronto. A partir desta pasta (controle-visao), para compilar e rodar:

    source /opt/ros/humble/setup.bash
    source .venv/bin/activate
    python3 -m colcon build --symlink-install
    source install/setup.bash
    ros2 launch ponte_visao_drone sistema.launch.py

Repare no "python3 -m colcon build", não apenas "colcon build": o executável `colcon`
instalado pelo apt tem seu próprio shebang fixo para o Python do sistema, então rodá-lo
direto ignora o venv ativado e gera os nós com o Python errado (sem acesso ao mediapipe).
Rodar como módulo do Python do venv ativado corrige isso.

EOF

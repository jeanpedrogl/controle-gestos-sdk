#!/usr/bin/env bash
# Abre tudo de uma vez: ambiente ROS2 + venv, compila se preciso e sobe o sistema.
# Uso: ./iniciar.sh   (Ctrl+C para encerrar)
cd "$(dirname "$(readlink -f "$0")")" || exit 1

source /opt/ros/humble/setup.bash || { echo "ROS2 Humble não encontrado"; exit 1; }
source .venv/bin/activate         || { echo "Falta o venv: rode ./setup_ambiente.sh"; exit 1; }

# Compila só se ainda não foi compilado (force com: ./iniciar.sh --build)
if [ ! -f install/setup.bash ] || [ "$1" = "--build" ]; then
    python3 -m colcon build --symlink-install || exit 1
fi
source install/setup.bash

echo "IP(s) deste computador para digitar no app do celular (porta 9090):"
hostname -I | tr ' ' '\n' | grep -v '^$' | sed 's/^/  /'
echo

exec ros2 launch ponte_visao_drone sistema.launch.py

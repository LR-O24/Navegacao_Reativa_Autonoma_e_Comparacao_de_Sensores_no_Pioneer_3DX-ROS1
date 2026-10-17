#!/usr/bin/env bash
# Configura ROS_MASTER_URI e ROS_IP para a rede multi-máquina.
# DEVE ser executado com "source":
#
#   source tools/ros_network.sh <IP_DO_ROBO> [interface]
#
# Funciona tanto no robô (onde o IP do robô é o próprio IP) quanto nos
# notebooks: o ROS_IP é descoberto automaticamente a partir da rota
# usada para alcançar o IP do robô. Use [interface] (ex.: wlp3s0) apenas
# se a detecção automática falhar.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Use: source tools/ros_network.sh <IP_DO_ROBO> [interface]"
  exit 1
fi

ROBOT_IP="$1"
IFACE="$2"

if [ -z "$ROBOT_IP" ]; then
  echo "Uso: source tools/ros_network.sh <IP_DO_ROBO> [interface]"
  return 1
fi

if [ -n "$IFACE" ]; then
  MY_IP=$(ip -4 addr show "$IFACE" | awk '/inet /{print $2}' | cut -d/ -f1 | head -n1)
else
  MY_IP=$(ip route get "$ROBOT_IP" 2>/dev/null \
          | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -n1)
fi

if [ -z "$MY_IP" ]; then
  echo "[ros_network] Não foi possível descobrir o IP local. Passe a interface como 2º argumento."
  return 1
fi

unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${ROBOT_IP}:11311"
export ROS_IP="${MY_IP}"

echo "[ros_network] ROS_MASTER_URI = ${ROS_MASTER_URI}"
echo "[ros_network] ROS_IP         = ${ROS_IP}"

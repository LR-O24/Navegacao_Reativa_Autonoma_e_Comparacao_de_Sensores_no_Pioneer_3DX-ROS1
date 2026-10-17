#!/usr/bin/env bash
# Diagnóstico rápido da rede ROS (rodar nos notebooks, após "source tools/ros_network.sh").
# Uso: tools/check_network.sh

if [ -z "$ROS_MASTER_URI" ]; then
  echo "ROS_MASTER_URI não definido. Rode: source tools/ros_network.sh <IP_DO_ROBO>"
  exit 1
fi

HOST=$(echo "$ROS_MASTER_URI" | sed -E 's#http://([^:/]+).*#\1#')
echo "Master: $ROS_MASTER_URI | ROS_IP local: $ROS_IP"
echo

echo "[1/4] ping ao robô..."
if ping -c 2 -W 2 "$HOST" > /dev/null 2>&1; then echo "  OK"; else echo "  FALHOU (mesma rede Wi-Fi? firewall?)"; fi

echo "[2/4] porta 11311 do master..."
if timeout 3 bash -c "</dev/tcp/$HOST/11311" 2>/dev/null; then echo "  OK"; else echo "  FALHOU (roscore está rodando no robô?)"; fi

echo "[3/4] nós e tópicos visíveis..."
if timeout 5 rosnode list > /tmp/_nodes.txt 2>&1; then
  echo "  nós: $(wc -l < /tmp/_nodes.txt)"
  echo "  tópicos: $(timeout 5 rostopic list 2>/dev/null | wc -l)"
else
  echo "  FALHOU: $(head -n1 /tmp/_nodes.txt)"
fi

echo "[4/4] taxas (3 s cada)..."
for T in /scan /RosAria/sonar /RosAria/pose; do
  echo "  $T:"
  timeout 3 rostopic hz "$T" 2>&1 | grep -m1 "average rate" | sed 's/^/    /' || true
done
echo
echo "Se o hz aparece mas os dados não chegam, confira ROS_IP e /etc/hosts (ver README)."

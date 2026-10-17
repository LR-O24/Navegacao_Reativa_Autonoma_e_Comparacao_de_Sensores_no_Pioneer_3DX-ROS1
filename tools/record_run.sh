#!/usr/bin/env bash
# Grava os tópicos necessários para a análise.
# Uso: tools/record_run.sh <nome> [duracao_s]
# Ex.:  tools/record_run.sh lidar_run1 180
#       tools/record_run.sh parado_lidar_sonar 30

NAME="${1:?Uso: tools/record_run.sh <nome> [duracao_s]}"
DUR="${2:-}"
DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
mkdir -p "$DIR"

ARGS=(-O "$DIR/${NAME}.bag" /scan /RosAria/sonar /RosAria/pose /RosAria/cmd_vel /tf)
[ -n "$DUR" ] && ARGS+=(--duration="$DUR")

echo "Gravando em $DIR/${NAME}.bag ..."
rosbag record "${ARGS[@]}"

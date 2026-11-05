#!/usr/bin/env bash
# Prepara el código de terceros de src/ (tras `vcs import`).
# 1) Parchea ldlidar_stl_ros2 para compilar con GCC 13 (ROS 2 Jazzy / Ubuntu 24.04).
#   - corrige la errata pthread_mutex_unloc -> pthread_mutex_unlock
#   - añade #include <pthread.h> (antes llegaba de forma indirecta)
# Es idempotente: se puede ejecutar varias veces.
# Uso: scripts/patch_ldlidar.sh   (tras `vcs import src < mtrb.repos`)
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
F="${WS_DIR}/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp"

if [ ! -f "$F" ]; then
  echo "No existe $F. Ejecuta antes: vcs import src < mtrb.repos" >&2
  exit 1
fi

sed -i.bak 's/pthread_mutex_unloc(/pthread_mutex_unlock(/g' "$F"
rm -f "${F}.bak"

if ! grep -q '#include <pthread.h>' "$F"; then
  sed -i.bak '1i #include <pthread.h>' "$F"
  rm -f "${F}.bak"
fi

echo "ldlidar_stl_ros2 parcheado: $F"

# m-explore-ros2 trae también multirobot_map_merge (fusión de mapas de varios robots), que no
# usamos y necesita image_geometry: se excluye de la compilación.
MERGE_DIR="${WS_DIR}/src/m-explore-ros2/map_merge"
if [ -d "$MERGE_DIR" ]; then
  touch "${MERGE_DIR}/COLCON_IGNORE"
  echo "multirobot_map_merge excluido (COLCON_IGNORE)"
fi

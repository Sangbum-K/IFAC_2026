#!/usr/bin/env bash
# make_raceline.sh — 맵 하나로 레이스라인 CSV를 한 방에 생성.
#   venv 없으면 만들고 -> 라이브러리 없으면 깔고 -> build_map.py 실행까지 자동.
#
# 사용법:
#   ./make_raceline.sh <맵이름> [--dir cw|ccw|auto]
# 예:
#   ./make_raceline.sh Spielberg_map
#   ./make_raceline.sh 새트랙 --dir cw
set -e

# --- 경로 (스크립트 위치 기준 자동 계산) --------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"
VENV="$WS_ROOT/.venv"
PY="$VENV/bin/python"

# --- 인자 확인 -----------------------------------------------------------
if [ -z "$1" ]; then
  echo "사용법: ./make_raceline.sh <맵이름> [옵션]"
  echo "예:    ./make_raceline.sh Spielberg_map --dir ccw"
  exit 1
fi
MAP="$1"; shift   # 나머지 인자는 build_map.py로 그대로 전달

# --- 1) venv 없으면 생성 -------------------------------------------------
if [ ! -x "$PY" ]; then
  echo "[setup] .venv 없음 -> 생성 중 ($VENV)"
  python3 -m venv "$VENV"
else
  echo "[setup] .venv 이미 있음 -> 재사용"
fi

# --- 2) 라이브러리 없으면 설치 (한 번만) --------------------------------
if ! PYTHONNOUSERSITE=1 "$PY" -c "import numpy, scipy, PIL, yaml, matplotlib, skimage" 2>/dev/null; then
  echo "[setup] 라이브러리 설치 중 (numpy scipy pillow pyyaml matplotlib scikit-image) ..."
  "$PY" -m pip install --quiet --upgrade pip
  "$PY" -m pip install --quiet numpy scipy pillow pyyaml matplotlib scikit-image
else
  echo "[setup] 라이브러리 이미 준비됨"
fi

# --- 3) 레이스라인 생성 --------------------------------------------------
echo "[run] 레이스라인 생성: $MAP $*"
cd "$SCRIPT_DIR"
PYTHONNOUSERSITE=1 "$PY" build_map.py "$MAP" "$@"

echo ""
echo "완료. 결과 확인:"
echo "  그림   : $SCRIPT_DIR/outputs/${MAP}_trajectory.png"
echo "  CSV    : planning_pkg/config/global_trajectory/${MAP}_iqp_*.csv"

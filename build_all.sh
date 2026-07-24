#!/usr/bin/env zsh
# build_all.sh — 코드 빌드 전용 (매번 실행).
#
# 시스템 의존성(rosdep·range_libc) 설치는 여기 없다 → 차량당 1회 ./setup_deps.sh.
# 이 스크립트는 순서대로 colcon build 만 한다. 의존성이 없으면 멈추고 안내한다.
#
# 사용법:
#   ./build_all.sh        # 실차: planning perception control integration hardware localization
#   ./build_all.sh sim    # 시뮬: hardware/localization 제외
#
# set -e/-u 는 쓰지 않는다 — ROS setup.zsh 가 unset 변수를 건드려 오작동하기 때문.

ROOT=${0:A:h}          # 스크립트 위치 = IFAC_ws 루트
PY=/usr/bin/python3    # ROS 노드가 쓰는 시스템 파이썬 (venv/conda/~.local 아님)

common_ws=(planning perception control integration)   # planning 먼저 (메시지 패키지)
car_ws=(hardware localization)

MODE=${1:-car}
case $MODE in
  sim) build_ws=($common_ws) ;;
  car) build_ws=($common_ws $car_ws) ;;
  *)   print "사용법: $0 [car|sim]"; exit 2 ;;
esac

fail() { print -P "%F{red}✗ $1%f"; exit 1; }
ok()   { print -P "%F{green}✓ $1%f"; }
info() { print -P "%F{cyan}▶ $1%f"; }

# ── numpy ABI 가드 (싸니까 매번) ────────────────────────────────────
# ~/.local 에 pip numpy 2.x 가 깔리면 scipy/ROS C확장이 ABI로 깨지고
# particle_filter 가 조용히 죽는다. 빌드 낭비 전에 먼저 막는다.
np=$($PY -c 'import numpy;print(numpy.__version__+" "+numpy.__file__)' 2>/dev/null) \
  || fail "numpy import 실패 — $PY 환경이 깨졌습니다"
np_ver=${np%% *}; np_file=${np#* }
if [[ $np_file == $HOME/.local/* || ${np_ver%%.*} != 1 ]]; then
  print -P "%F{red}⚠ numpy 오염: $np%f"
  print   "  고치기: $PY -m pip uninstall -y numpy   # → apt 1.x 복귀 후 재실행"
  fail "numpy 가 apt 1.x 가 아님 (ROS C확장 ABI 붕괴)"
fi
ok "numpy $np_ver (apt)"

# ── ROS 2 Humble ────────────────────────────────────────────────────
source /opt/ros/humble/setup.zsh || fail "ROS 2 Humble source 실패"

# ── 의존성 게이트 (설치 안 됐으면 setup_deps 로 안내하고 멈춤) ──────
# 매번 설치하지 않고, 있는지만 빠르게 확인한다.
if [[ $MODE == car ]]; then
  $PY -c 'import range_libc, tf_transformations, transforms3d' 2>/dev/null \
    || fail "particle_filter 의존성 미설치 — 먼저 './setup_deps.sh' 를 한 번 실행하세요 (차량당 1회)"
  ok "PF 의존성 확인 (range_libc / tf_transformations / transforms3d)"
fi

# ── colcon build (순서 보장 · --base-paths src · 빌드 후 source) ─────
# --base-paths src : 워크스페이스 루트 잔재(perception)나 옆 스택(unicorn) 오탐 차단.
for ws in $build_ws; do
  info "colcon build: ${ws}_ws"
  cd $ROOT/${ws}_ws || fail "${ws}_ws 디렉토리 없음"
  colcon build --base-paths src || fail "${ws}_ws 빌드 실패"
  source install/setup.zsh || fail "${ws}_ws source 실패"
done

# ── 안내 ────────────────────────────────────────────────────────────
print ""
ok "빌드 완료 (mode=$MODE)"
print "실행 전 새 터미널에서 아래를 source 하세요:"
print "  source /opt/ros/humble/setup.zsh"
for ws in $build_ws; do print "  source $ROOT/${ws}_ws/install/setup.zsh"; done
[[ $MODE == car ]] && print "\n그다음:  ros2 launch integration_pkg real_car.launch.py"

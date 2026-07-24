#!/usr/bin/env zsh
# setup_deps.sh — 차량(머신)당 최초 1회: 시스템 의존성 설치.
#
# rosdep 의존성 + range_libc(소스 빌드) + numpy ABI 검사.
# 코드만 고칠 땐 필요 없다 → 그건 ./build_all.sh 가 담당.
# 의존성을 새로 추가했거나 머신을 새로 세팅할 때만 다시 실행하면 된다.
#
# 사용법:
#   ./setup_deps.sh        # 실차 (range_libc 포함)
#   ./setup_deps.sh sim    # 시뮬 (range_libc 불필요)
#
# set -e/-u 는 쓰지 않는다 — ROS setup.zsh 가 unset 변수를 건드려 오작동하기 때문.

ROOT=${0:A:h}
PY=/usr/bin/python3

common_ws=(planning perception control integration)
car_ws=(hardware localization)

MODE=${1:-car}
case $MODE in
  sim) build_ws=($common_ws);          need_rangelibc=0 ;;
  car) build_ws=($common_ws $car_ws);  need_rangelibc=1 ;;
  *)   print "사용법: $0 [car|sim]"; exit 2 ;;
esac

fail() { print -P "%F{red}✗ $1%f"; exit 1; }
ok()   { print -P "%F{green}✓ $1%f"; }
info() { print -P "%F{cyan}▶ $1%f"; }

# ── numpy ABI 가드 ──────────────────────────────────────────────────
info "numpy 무결성 확인 (apt 1.x 고정이어야 함)"
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

# ── rosdep (빌드할 워크스페이스 전부, 한 번에) ──────────────────────
# tf_transformations, transforms3d(pip), laser_proc, serial_driver 등 자동 설치.
# 모든 src 를 한 호출에 넘겨야 크로스-ws 소스 의존(f1tenth_perception_msgs 등)을
# "로컬 소스"로 인식해 헛되이 설치하려 들지 않는다.
if command -v rosdep >/dev/null; then
  src_paths=()
  for ws in $build_ws; do src_paths+=($ROOT/${ws}_ws/src); done
  info "rosdep install (${#build_ws} workspaces) — apt는 sudo 비밀번호를 물어봄"
  rosdep install --from-paths $src_paths --ignore-src -r -y \
    || print -P "%F{yellow}  ! rosdep 일부 실패 (오프라인/미초기화? 'sudo rosdep init && rosdep update')%f"
  # rosdep 의 pip installer 가 numpy 를 오염시켰는지 재확인
  np2=$($PY -c 'import numpy;print(numpy.__file__)' 2>/dev/null)
  [[ $np2 == $HOME/.local/* ]] && \
    fail "rosdep 이 ~/.local numpy 를 설치함 — '$PY -m pip uninstall -y numpy' 후 재실행"
  ok "rosdep 완료"
else
  fail "rosdep 미설치 — 'sudo apt install python3-rosdep2 && sudo rosdep init && rosdep update' 후 재실행"
fi

# ── range_libc (rosdep 밖 유일한 진짜 수동 의존, 실차만) ────────────
if (( need_rangelibc )); then
  if $PY -c 'import range_libc' 2>/dev/null; then
    ok "range_libc 이미 설치됨"
  elif [[ -d ~/range_libc/pywrapper ]]; then
    info "range_libc 없음 — 소스에서 빌드 (~/range_libc, CPU)"
    ( cd ~/range_libc/pywrapper && $PY setup.py install --user ) || fail "range_libc 빌드 실패"
    $PY -c 'import range_libc' 2>/dev/null && ok "range_libc 빌드 완료" \
      || fail "range_libc 빌드했으나 import 실패"
  else
    fail "~/range_libc 없음. 먼저:  git clone https://github.com/f1tenth/range_libc ~/range_libc"
  fi

  # PF 런타임 의존성 최종 검증 — '빌드는 되는데 실행 시 죽는' 클래스 사전 차단
  info "particle_filter 런타임 의존성 검증"
  $PY -c 'import range_libc, tf_transformations, transforms3d' 2>/dev/null \
    && ok "range_libc / tf_transformations / transforms3d OK" \
    || fail "PF 의존성 import 실패 — 위 rosdep/range_libc 로그 확인"
fi

# ── 안내 ────────────────────────────────────────────────────────────
print ""
ok "의존성 셋업 완료 (mode=$MODE)"
print "이제 빌드하세요:  ./build_all.sh $MODE"

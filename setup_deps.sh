#!/usr/bin/env zsh
# setup_deps.sh — 차량(머신)당 최초 1회: 시스템 의존성 설치.
#
# rosdep 의존성 + RangeLib C++ 헤더 + numpy ABI 검사.
# 코드만 고칠 땐 필요 없다 → 그건 ./build_all.sh 가 담당.
# 의존성을 새로 추가했거나 머신을 새로 세팅할 때만 다시 실행하면 된다.
#
# 사용법:
#   ./setup_deps.sh        # 실차 (RangeLib C++ 헤더 포함)
#   ./setup_deps.sh sim    # 시뮬 (hardware_ws 만 뺀다 — RangeLib 은 sim 도 필요)
#
# set -e/-u 는 쓰지 않는다 — ROS setup.zsh 가 unset 변수를 건드려 오작동하기 때문.

ROOT=${0:A:h}
PY=/usr/bin/python3

common_ws=(planning perception control integration)
car_ws=(hardware localization)

MODE=${1:-car}
# sim 도 localization 을 빌드한다 — sim_race.launch.py pf:=true 가 실차와 같은
# particle_filter_cpp 를 gym 위에 올린다. build_all.sh 의 build_ws 와 반드시
# 같아야 한다: 여기서 localization 을 빼면 RangeLib 헤더가 안 깔린 채
# ./build_all.sh sim 이 CMake configure 에서 죽고, 안내대로 setup_deps 를 다시
# 돌려도 여전히 안 깔려 빠져나갈 수 없다.
case $MODE in
  sim) build_ws=($common_ws localization); need_rangelibc=1 ;;
  car) build_ws=($common_ws $car_ws);      need_rangelibc=1 ;;
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
# laser_proc, serial_driver 등 ROS 패키지 의존성을 자동 설치한다.
# 모든 src 를 한 호출에 넘겨야 크로스-ws 소스 의존(f1tenth_perception_msgs 등)을
# "로컬 소스"로 인식해 헛되이 설치하려 들지 않는다.
if command -v rosdep >/dev/null; then
  src_paths=()
  for ws in $build_ws; do src_paths+=($ROOT/${ws}_ws/src); done
  info "rosdep install (${#build_ws} workspaces) — apt는 sudo 비밀번호를 물어봄"
  rosdep_ok=1
  rosdep install --from-paths $src_paths --ignore-src -r -y || rosdep_ok=0
  # rosdep 의 pip installer 가 numpy 를 오염시켰는지 재확인
  np2=$($PY -c 'import numpy;print(numpy.__file__)' 2>/dev/null)
  [[ $np2 == $HOME/.local/* ]] && \
    fail "rosdep 이 ~/.local numpy 를 설치함 — '$PY -m pip uninstall -y numpy' 후 재실행"
  if (( rosdep_ok )); then
    ok "rosdep 완료"
  else
    # 예전엔 여기서도 "✓ rosdep 완료"를 찍었다. 그래서 sudo 비밀번호를 못 받아
    # apt 가 통째로 실패한 날에도 성공처럼 보였고, monitor_pkg 오버레이가
    # 조용히 빠진 채로 몇 주를 갔다. 실패는 실패라고 말한다.
    print -P "%F{yellow}  ! rosdep 일부 실패 — 위 apt 로그 확인%f"
    print   "    비밀번호 입력이 필요하다면 이 스크립트를 실제 터미널에서 실행할 것"
    print   "    (오프라인/미초기화면: sudo rosdep init && rosdep update)"
    print   "    아래 검증 단계에서 실제로 무엇이 빠졌는지 걸러낸다."
  fi
else
  fail "rosdep 미설치 — 'sudo apt install python3-rosdep2 && sudo rosdep init && rosdep update' 후 재실행"
fi

# ── RViz 2D 오버레이 (monitor_pkg) ──────────────────────────────────
# monitor_pkg/package.xml 에 선언돼 있으니 rosdep 이 잡아야 정상인데, 위 rosdep
# 실패는 경고만 내고 넘어가므로 조용히 빠질 수 있다. 실제로 그렇게 빠져서
# monitor_pkg/CMakeLists 의 find_package(... QUIET) 가 monitor_node 를 통째로
# 스킵했고, "빌드 성공했는데 랩타임/CTE 오버레이가 안 뜬다" 로 나타났다.
# 여기서 못 박아 둔다 — 빌드가 아니라 셋업 단계에서 걸러야 하는 문제다.
info "RViz 2D 오버레이 (monitor_pkg 랩타임/CTE 표시) 확인"
missing_overlay=()
for p in rviz_2d_overlay_msgs rviz_2d_overlay_plugins; do
  [[ -d /opt/ros/humble/share/$p ]] || missing_overlay+=($p)
done
if (( ${#missing_overlay} )); then
  print -P "%F{yellow}  ! 누락: ${missing_overlay[*]}%f"
  print   "    설치:  sudo apt install ${${missing_overlay[@]/#/ros-humble-}//_/-}"
  print   "    (없으면 monitor_node 가 빌드 자체를 스킵하고 오버레이가 전부 안 뜬다)"
  fail "monitor_pkg 오버레이 의존성 누락"
fi
ok "rviz_2d_overlay_msgs / rviz_2d_overlay_plugins OK"

# ── RangeLib C++ headers (rosdep 밖 수동 의존, 실차만) ──────────────
if (( need_rangelibc )); then
  if [[ -f ~/range_libc/includes/RangeLib.h ]]; then
    ok "RangeLib C++ headers 확인 (~/range_libc/includes/RangeLib.h)"
  else
    fail "~/range_libc 없음. 먼저:  git clone https://github.com/f1tenth/range_libc ~/range_libc"
  fi
fi

# ── 안내 ────────────────────────────────────────────────────────────
print ""
ok "의존성 셋업 완료 (mode=$MODE)"
print "이제 빌드하세요:  ./build_all.sh $MODE"

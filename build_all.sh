#!/usr/bin/env zsh
# build_all.sh — 코드 빌드 전용 (매번 실행).
#
# 시스템 의존성(rosdep·RangeLib) 설치는 여기 없다 → 차량당 1회 ./setup_deps.sh.
# 이 스크립트는 순서대로 colcon build 만 한다. 의존성이 없으면 멈추고 안내한다.
#
# 사용법:
#   ./build_all.sh        # 실차: planning perception control integration hardware localization
#   ./build_all.sh sim    # 시뮬: sim(gym) + planning perception control integration localization
#                         #       (hardware 만 제외. gym 도 여기서 같이 빌드된다)
#
# set -e/-u 는 쓰지 않는다 — ROS setup.zsh 가 unset 변수를 건드려 오작동하기 때문.

ROOT=${0:A:h}          # 스크립트 위치 = IFAC_ws 루트
PY=/usr/bin/python3    # ROS 노드가 쓰는 시스템 파이썬 (venv/conda/~.local 아님)

common_ws=(planning perception control integration)   # planning 먼저 (메시지 패키지)
car_ws=(hardware localization)

MODE=${1:-car}
case $MODE in
  # sim_ws(gym)를 맨 앞에 둔다 — 우리 패키지에 대한 의존이 없어 순서는 자유지만
  # run_sim.sh 의 source 순서와 맞춰 둔다. 예전에는 여기서 빼놔서 "gym은 따로
  # colcon build" 라는 수동 단계가 남아 있었고, 새 머신에서 그걸 빠뜨리면
  # run_sim.sh 가 저장소 밖 사본으로 폴백하거나 아예 못 찾고 죽었다.
  # f1tenth_gym_ros 는 ament_python 이라 install/ 에 절대경로가 박힌다 —
  # 빌드 산출물을 저장소에 넣을 수 없으므로 머신마다 이 단계가 필요하다(~1초).
  # localization은 sim에도 필요하다 — sim_race.launch.py pf:=true 가
  # particle_filter를 gym 위에 올려 실차와 같은 위치추정 오차를 재현한다.
  sim) build_ws=(sim $common_ws localization) ;;
  car) build_ws=($common_ws $car_ws) ;;
  *)   print "사용법: $0 [car|sim]"; exit 2 ;;
esac

fail() { print -P "%F{red}✗ $1%f"; exit 1; }
ok()   { print -P "%F{green}✓ $1%f"; }
info() { print -P "%F{cyan}▶ $1%f"; }

# ── numpy ABI 가드 (싸니까 매번) ────────────────────────────────────
# ~/.local 에 pip numpy 2.x 가 깔리면 scipy/ROS C확장이 ABI로 깨지고
# ROS Python 노드(planning / monitor / integration 스크립트)가 조용히 죽는다.
# 빌드 낭비 전에 먼저 막는다. PF 는 C++ 로 옮겨갔으니(2026-08-11 머지) 더는
# 이 가드가 지키는 대상이 아니지만, 파이썬 노드가 남아 있어 가드는 그대로 둔다.
#
# ⚠️ 단, 시뮬 데스크탑은 user-site numpy 를 **필요로 한다**: f110_gym 이 쓰는
# numba 와 build_map.py 가 쓰는 skimage/scipy 가 거기에만 있고 그 numpy 에 맞춰
# 빌드돼 있다. PYTHONNOUSERSITE=1 로 apt 1.21 을 강제하면 gym 이 안 뜬다.
# 그래서 sim 모드에서는 경고만 하고 진행한다.
np=$($PY -c 'import numpy;print(numpy.__version__+" "+numpy.__file__)' 2>/dev/null) \
  || fail "numpy import 실패 — $PY 환경이 깨졌습니다"
np_ver=${np%% *}; np_file=${np#* }
if [[ $np_file == $HOME/.local/* || ${np_ver%%.*} != 1 ]]; then
  if [[ $MODE == sim ]]; then
    print -P "%F{yellow}⚠ numpy 가 apt 1.x 가 아님: $np%f"
    print   "  sim 모드라 계속합니다 (gym numba / build_map skimage 가 이 스택을 씁니다)."
    print   "  실차 빌드(./build_all.sh)에서는 이게 fail 입니다."
  else
    print -P "%F{red}⚠ numpy 오염: $np%f"
    print   "  고치기: $PY -m pip uninstall -y numpy   # → apt 1.x 복귀 후 재실행"
    fail "numpy 가 apt 1.x 가 아님 (ROS C확장 ABI 붕괴)"
  fi
else
  ok "numpy $np_ver (apt)"
fi

# ── ROS 2 Humble ────────────────────────────────────────────────────
source /opt/ros/humble/setup.zsh || fail "ROS 2 Humble source 실패"

# ── 의존성 게이트 (설치 안 됐으면 setup_deps 로 안내하고 멈춤) ──────
# 매번 설치하지 않고, 있는지만 빠르게 확인한다.
# sim 모드도 localization을 빌드하므로(pf:=true) 모드가 아니라 build_ws로 판정한다.
# PF 가 C++ 가 된 뒤로 이건 런타임 임포트가 아니라 *컴파일* 의존이다 —
# 헤더가 없으면 particle_filter_cpp 가 CMake configure 단계에서 죽는다.
if (( ${build_ws[(I)localization]} )); then
  # 위치를 $HOME 에 못박지 않는다. setup_deps.sh · particle_filter_cpp/CMakeLists.txt
  # 와 **같은 순서**로 찾고, 찾은 값을 RANGE_LIBC 로 export 해서 CMake 가 반드시
  # 같은 체크아웃을 쓰게 한다 (여기서 OK 인데 CMake 는 다른 걸 잡는 일 방지).
  rangelibc_root=
  for c in ${RANGE_LIBC:+$RANGE_LIBC} $HOME/range_libc $ROOT/range_libc \
           ${ROOT:h}/range_libc /opt/range_libc; do
    [[ -f $c/includes/RangeLib.h ]] && { rangelibc_root=${c:A}; break }
  done
  [[ -n $rangelibc_root ]] \
    || fail "RangeLib C++ 헤더 없음 — 먼저 './setup_deps.sh $MODE' 를 실행하세요 (머신당 1회).
  이미 어딘가에 받아 뒀다면:  RANGE_LIBC=<경로> $0 $MODE"
  export RANGE_LIBC=$rangelibc_root
  ok "PF 의존성 확인 (RangeLib C++ headers: $rangelibc_root)"
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
if [[ $MODE == car ]]; then
  print "\n그다음:  ros2 launch integration_pkg real_car.launch.py"
else
  print "\n그다음:  ./run_sim.sh <맵이름>   (또는 원커맨드 simm <맵이름>)"
fi

# 성공 종료를 명시한다. 예전에는 위 안내가 `[[ $MODE == car ]] && print ...` 라
# sim 모드에서 마지막 명령의 상태 1이 그대로 스크립트 종료코드가 됐고,
# 이걸 `|| fail` 로 감싸 호출하는 쪽(simm.sh)이 성공한 빌드를 실패로 봤다.
exit 0

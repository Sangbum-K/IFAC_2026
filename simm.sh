#!/usr/bin/env zsh
# simm.sh — 맵 이름 하나로 "레이스라인 생성 -> 전체 빌드 -> 시뮬 실행"까지.
#
#   simm <맵이름> [옵션]
#
# 예:
#   simm my_track0807             maps/my_track0807.{pgm,yaml} 로 CSV 뽑고 빌드하고 실행
#   simm my_track0807 --regen     CSV 가 이미 있어도 지금 상수로 다시 생성
#   simm my_track0807 --dir cw    시계방향 트랙 (기본 ccw)
#   simm my_track0807 --no-build  colcon build 건너뜀 (CSV 가 이미 설치된 상태에서 재실행)
#   simm my_track0807 --gt        위치추정을 gym GT 로 (pf:=false)
#   simm my_track0807 --plain     env:=sim 프로파일 (gym 기본 차량물리)
#   simm my_track0807 --no-rviz   RViz 없이
#   simm my_track0807 suffix:=... 그 외 인자는 run_sim.sh -> ros2 launch 로 그대로 전달
#
# 새 트랙을 넣는 절차는 이게 전부다:
#   1) ~/IFAC_ws/maps/ 에 <맵>.pgm(또는 .png) + <맵>.yaml 을 넣는다
#      (yaml 의 image: 줄이 실제 파일명과 같아야 한다)
#   2) simm <맵>
#
# 순서가 "생성 -> 빌드 -> 실행"인 이유: raceline CSV 는 소스 트리
# (planning_pkg/config/global_trajectory/)에 생기고 colcon build 가 install/ 로
# **복사**한다. 빌드를 먼저 하면 방금 만든 CSV 가 설치되지 않아 런타임 노드가
# 못 찾는다. 맵 파일 자체는 패키지에 설치되지 않으므로(launch 가 maps/ 를 직접
# 읽는다) 맵만 고쳤을 때는 --no-build 로 충분하다.

setopt noequals    # zsh 의 =cmd 확장이 rviz:=false 같은 인자를 망가뜨린다

ROOT=${0:A:h}
MAPS_DIR=$ROOT/maps
BUILD_MAP_DIR=$ROOT/planning_ws/src/planning_pkg/scripts/global_traj_sim
GT_DIR=$ROOT/planning_ws/src/planning_pkg/config/global_trajectory
PY=/usr/bin/python3

fail() { print -P "%F{red}✗ $1%f"; exit 1 }
info() { print -P "%F{cyan}▶ $1%f" }
ok()   { print -P "%F{green}✓ $1%f" }

list_maps() {
  local f names=()
  for f in $MAPS_DIR/*.yaml(N); do names+=(${${f:t}:r}); done
  print -- "${(j:, :)names}"
}

# ── 인자 ────────────────────────────────────────────────────────────
map=
dir_arg=ccw
do_build=1
regen=0
passthru=()
while (( $# )); do
  case $1 in
    --regen)     regen=1 ;;
    --no-build)  do_build=0 ;;
    --dir)       shift; dir_arg=$1 ;;
    --dir=*)     dir_arg=${1#--dir=} ;;
    -h|--help)   awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' $0
                 print "사용 가능한 맵: $(list_maps)"
                 exit 0 ;;
    --*|*:=*)    passthru+=("$1") ;;
    *)           if [[ -z $map ]]; then map=$1; else passthru+=("$1"); fi ;;
  esac
  shift
done

[[ -n $map ]] || fail "맵 이름이 필요합니다.
  사용법: simm <맵이름> [옵션]      (자세히: simm --help)
  사용 가능한 맵: $(list_maps)"

# ── 0) 맵 파일 확인 (yaml + image 짝) ───────────────────────────────
map_yaml=$MAPS_DIR/$map.yaml
[[ -f $map_yaml ]] || fail "맵 yaml 이 없습니다: $map_yaml
  맵은 이 폴더 한 곳에 둡니다 (mapp <이름>이 저장하는 곳).
  사용 가능한 맵: $(list_maps)"

map_img=$($PY - "$map_yaml" <<'PYEOF'
import os, sys, yaml
path = sys.argv[1]
meta = yaml.safe_load(open(path, encoding='utf-8')) or {}
image = (meta.get('image') or '').strip()
if not image:
    sys.exit('yaml 에 image: 항목이 없습니다')
if not os.path.isabs(image):
    image = os.path.join(os.path.dirname(path), image)
print(os.path.abspath(image))
PYEOF
) || fail "맵 yaml 을 읽지 못했습니다: $map_yaml"
[[ -f $map_img ]] || fail "yaml 이 가리키는 이미지가 없습니다: $map_img
  $map_yaml 의 image: 줄을 실제 파일명과 맞추세요."
ok "맵: $map ($map_img:t)"

# ── 1) 레이스라인 CSV (없으면 생성, --regen 이면 항상) ──────────────
existing=($GT_DIR/${map}_iqp_*.csv(N))
if (( regen )) || (( ! ${#existing} )); then
  if (( ${#existing} )); then
    info "레이스라인 재생성 (--regen): 기존 ${#existing}개 교체"
    rm -f $GT_DIR/${map}_iqp_*.csv $GT_DIR/${map}_sp_*.csv
  else
    info "레이스라인 CSV 없음 -> build_map.py 로 생성 (--dir $dir_arg)"
  fi
  # build_map.py 는 numpy/scipy/skimage/matplotlib/PIL 을 쓴다. 이 기계에서는
  # skimage 가 user-site 에만 있으므로 PYTHONNOUSERSITE 를 걸면 안 된다
  # (걸면 apt numpy 1.21 로 내려가면서 skimage 를 통째로 잃는다).
  $PY -c 'import numpy, scipy, PIL, yaml, matplotlib, skimage' 2>/dev/null \
    || fail "build_map.py 의존성 부족 — 설치: $PY -m pip install --user numpy scipy pillow pyyaml matplotlib scikit-image"
  ( cd $BUILD_MAP_DIR && $PY build_map.py "$map" --dir "$dir_arg" ) \
    || fail "레이스라인 생성 실패 ($map). 맵 이미지/해상도를 확인하세요."
  generated=($GT_DIR/${map}_iqp_*.csv(N))
  (( ${#generated} )) || fail "build_map.py 가 끝났는데 CSV 가 없습니다: $GT_DIR/${map}_iqp_*.csv"
  ok "레이스라인 생성: ${generated[1]:t}"
  do_build=1     # 새 CSV 는 install/ 로 복사돼야 런타임이 읽는다
else
  ok "레이스라인 재사용: ${existing[1]:t}  (다시 뽑으려면 --regen)"
fi

# ── 2) 전체 빌드 (CSV 를 install/ 로 복사하는 단계이기도 하다) ──────
if (( do_build )); then
  info "전체 빌드: ./build_all.sh sim"
  $ROOT/build_all.sh sim || fail "빌드 실패"
else
  info "빌드 건너뜀 (--no-build)"
fi

# ── 3) 실행 ─────────────────────────────────────────────────────────
info "시뮬 실행: $map"
exec $ROOT/run_sim.sh "$map" "${passthru[@]}"

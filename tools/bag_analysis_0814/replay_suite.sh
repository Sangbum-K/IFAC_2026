#!/usr/bin/env bash
# ② (side_clears 접근 구간) 의 실노드 open-loop A/B.
#   new_r1  : 수정 포함 빌드
#   base_r1 : 수정을 stash 로 뺀 빌드
#   new2_r1 : 수정 포함 재실행 (런 간 노이즈 추정용)
# stash 는 base 실행 직후 되돌리고, 실패해도 trap 으로 복구한다.
set +u
cd /home/sangbum/IFAC_ws || exit 1
WS=/home/sangbum/IFAC_ws/planning_ws
SRC="planning_ws/src/planning_pkg/src/spliner_node.cpp planning_ws/src/planning_pkg/include/planning_pkg/spliner_node.hpp"
STASHED=0
restore() {
  if [ "$STASHED" = "1" ]; then
    git stash pop && STASHED=0
    (cd "$WS" && source /opt/ros/humble/setup.bash && colcon build --packages-select planning_pkg > /dev/null 2>&1)
  fi
}
trap restore EXIT

build() { (cd "$WS" && source /opt/ros/humble/setup.bash && colcon build --packages-select planning_pkg 2>&1 | tail -2); }

echo "=== new_r1 ==="
build
bash tools/bag_analysis_0814/replay_ab.sh new_r1 1.0

echo "=== base_r1 (수정 제거) ==="
git stash push -m "sideclears_ab_baseline" -- $SRC && STASHED=1
build
bash tools/bag_analysis_0814/replay_ab.sh base_r1 1.0
restore

echo "=== new2_r1 (노이즈 추정) ==="
bash tools/bag_analysis_0814/replay_ab.sh new2_r1 1.0
echo "=== suite done ==="
git status --short

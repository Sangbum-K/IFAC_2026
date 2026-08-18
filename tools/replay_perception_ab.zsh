#!/usr/bin/env zsh
# 백을 재생해 obstacle_detector_node 출력을 다시 뽑는다 (A/B 용).
#   usage: [DUR=<sec>] replay_perception_ab.zsh <bag_dir> <out_prefix>
BAG=$1; OUT=$2; DUR=${DUR:-0}
source /opt/ros/humble/setup.zsh
source /home/miru/IFAC_ws/perception_ws/install/setup.zsh
source /home/miru/IFAC_ws/planning_ws/install/setup.zsh
SHARE=/home/miru/IFAC_ws/perception_ws/install/perception_pkg/share/perception_pkg
rm -rf $OUT $OUT.*.log
ros2 run perception_pkg obstacle_detector_node --ros-args \
  --params-file $SHARE/config/params.yaml \
  --params-file $SHARE/config/environments/real_car.yaml \
  -p use_sim_time:=true > $OUT.node.log 2>&1 &
NODE_PID=$!
sleep 3
ros2 bag record -o $OUT /perception/obstacles > $OUT.rec.log 2>&1 &
REC_PID=$!
sleep 2
PLAY="ros2 bag play $BAG --clock 200 --topics /scan /tf /tf_static /map /global_waypoints"
if [[ $DUR -gt 0 ]]; then timeout $DUR ${=PLAY} > $OUT.play.log 2>&1
else ${=PLAY} > $OUT.play.log 2>&1; fi
sleep 3
kill -INT $REC_PID 2>/dev/null; sleep 4
kill -INT $NODE_PID 2>/dev/null; sleep 2
pkill -f obstacle_detector_node 2>/dev/null
echo "done: $OUT"

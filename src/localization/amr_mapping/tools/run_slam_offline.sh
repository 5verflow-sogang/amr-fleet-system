#!/bin/bash
# 녹화한 rosbag을 재생하며 slam_toolbox로 지도를 만든다.  bash run_slam_offline.sh <해상도> <지도 이름>
RES=$1
NAME=$2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
# 이전 실행이 남긴 노드가 TF·/map을 같이 내지 않게 정리한다
pkill -f async_slam_toolbox_node; pkill -f tf_jump_logger; pkill -f "bag play"; sleep 2
BAG=/bags/warehouse_v1_mapping
OUT=/ros2_ws/src/localization/amr_mapping/maps/$NAME
LOG=/bags/slam_$NAME.log
: > $LOG
DEF=$(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/config/mapper_params_online_async.yaml
OURS=/ros2_ws/install/amr_mapping/share/amr_mapping/config/slam_toolbox_mapping.yaml
echo "[$(date +%T)] slam 시작 resolution=$RES" >> $LOG
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -r __node:=slam_toolbox \
  --params-file $DEF --params-file $OURS \
  -p resolution:=$RES -p odom_frame:=amr_01/odom -p base_frame:=amr_01/base_footprint \
  -p scan_topic:=/amr_01/scan -p use_sim_time:=true > /bags/slam_node_$NAME.log 2>&1 &
SLAM=$!
python3 "$(dirname "$0")/tf_jump_logger.py" >> $LOG 2>&1 &
TFL=$!
sleep 6
echo "[$(date +%T)] bag 재생" >> $LOG
# 녹화된 /clock을 그대로 쓴다. 지도 v1 bag(sim 거의 실시간 녹화)에서 1배속·2배속 결과가 같았다 (10/7)
ros2 bag play $BAG --rate ${RATE:-2.0} > /bags/play_$NAME.log 2>&1
sleep 10
echo "[$(date +%T)] 지도 저장 → $OUT" >> $LOG
ros2 run nav2_map_server map_saver_cli -f $OUT --ros-args -p save_map_timeout:=10.0 >> $LOG 2>&1
kill -INT $TFL; sleep 2
kill $SLAM 2>/dev/null; pkill -f async_slam_toolbox_node; pkill -f tf_jump_logger; sleep 2
grep -iE "loop|clos" /bags/slam_node_$NAME.log | head -20 >> $LOG
echo "DONE" >> $LOG

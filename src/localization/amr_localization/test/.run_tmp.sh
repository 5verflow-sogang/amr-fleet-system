#!/bin/bash
# 임시 시험 실행기 (커밋하지 않음). exec 세션과 분리해서 돈다.
source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash
rm -rf /tmp/eval_amcl /tmp/run.done
ros2 run amr_localization eval_node --ros-args -p use_sim_time:=true -p robot:=turtlebot4 \
  -p gt_topic:=/sim_ground_truth_pose -p estimate_topic:=/amcl_pose -p estimate_type:=pose \
  -p output_dir:=/tmp/eval_amcl > /tmp/eval.log 2>&1 &
EV=$!
sleep 3
python3 /ros2_ws/src/localization/amr_localization/test/.drive_tmp.py > /tmp/drive.log 2>&1
echo "drive exit=$?" >> /tmp/drive.log
kill -INT $EV; wait $EV
echo done > /tmp/run.done

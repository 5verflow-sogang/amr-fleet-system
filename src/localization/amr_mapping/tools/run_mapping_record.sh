#!/bin/bash
# 지도 v1 매핑 주행을 rosbag으로 녹화한다 (작업 컨테이너 안에서 실행)
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
# 이전 시험이 남긴 sim·브리지가 있으면 같은 토픽을 두 번 내서 bag에 메시지가 중복된다 (10/7 녹화에서 겪음)
pkill -f "[i]gn gazebo"; pkill -f "[p]arameter_bridge"; pkill -f "[r]obot_state_publisher"; pkill -f "[s]can_filter"; pkill -f "[i]mu_filter"; sleep 2
BAG=/bags/warehouse_v1_mapping
rm -rf "$BAG"
LOG=/bags/run_record.log
: > $LOG
echo "[$(date +%T)] sim 시작" >> $LOG
ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false enable_camera:=false > /bags/sim.log 2>&1 &
SIM=$!
sleep 20
ros2 launch amr_odometry sensor_filters.launch.py > /bags/filters.log 2>&1 &
FILT=$!
sleep 6   # IMU 바이어스 보정 (정지 2 s)
echo "[$(date +%T)] 녹화 시작" >> $LOG
ros2 bag record -o "$BAG" \
  /clock /tf /tf_static \
  /amr_01/scan /amr_01/scan_raw /amr_01/imu/data /amr_01/imu/data_raw \
  /amr_01/joint_states /amr_01/diff_drive/odom /amr_01/ground_truth/odom /amr_01/cmd_vel \
  /amr_01/robot_description > /bags/bag.log 2>&1 &
BAGP=$!
sleep 4
ros2 run amr_mapping map_route_driver >> $LOG 2>&1
sleep 3
kill -INT $BAGP; sleep 5
echo "[$(date +%T)] 녹화 끝" >> $LOG
kill $FILT $SIM 2>/dev/null; sleep 3
pkill -f "ign gazebo" 2>/dev/null
ros2 bag info "$BAG" >> $LOG 2>&1
echo "DONE" >> $LOG

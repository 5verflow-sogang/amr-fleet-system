# 임시 주행 스크립트 (커밋하지 않음): 초기 위치 지정 → 정지 → 트인 방향으로 회전 → 직진 → 정지
import math, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = Node('drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
gt = {}; scan = {}
n.create_subscription(Odometry, '/sim_ground_truth_pose', lambda m: gt.update(m=m), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/scan', lambda m: scan.update(m=m), qos_profile_sensor_data)
cmd = n.create_publisher(Twist, '/cmd_vel', 10); ip = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
while 'm' not in gt or 'm' not in scan: rclpy.spin_once(n, timeout_sec=0.1)
def now(): return n.get_clock().now().nanoseconds * 1e-9
def run(v, w, dur):
    t0 = now()
    while now() - t0 < dur:
        tw = Twist(); tw.linear.x = v; tw.angular.z = w; cmd.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
    cmd.publish(Twist())
g = gt['m'].pose.pose
p = PoseWithCovarianceStamped(); p.header.frame_id = 'map'; p.header.stamp = gt['m'].header.stamp
p.pose.pose = g; p.pose.covariance[0] = p.pose.covariance[7] = 0.05; p.pose.covariance[35] = 0.05
for _ in range(3): ip.publish(p); rclpy.spin_once(n, timeout_sec=0.2)
s = scan['m']; i = max(range(len(s.ranges)), key=lambda k: s.ranges[k] if math.isfinite(s.ranges[k]) else 0)
ang = s.angle_min + i * s.angle_increment; ang = math.atan2(math.sin(ang), math.cos(ang))
print(f'GT 시작 ({g.position.x:.2f}, {g.position.y:.2f}), 가장 트인 방향 {math.degrees(ang):.0f}도 {s.ranges[i]:.1f}m', flush=True)
run(0.0, 0.0, 4.0)                                  # 정지
run(0.0, 0.5 if ang > 0 else -0.5, abs(ang) / 0.5)  # 회전
run(0.0, 0.0, 1.0)
run(0.25, 0.0, min(8.0, (s.ranges[i] - 0.6) / 0.25))  # 직진 (벽 0.6m 앞까지)
run(0.0, 0.0, 3.0)                                  # 정지
print('주행 끝', flush=True)

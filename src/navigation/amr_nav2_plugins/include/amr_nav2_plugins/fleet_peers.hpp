#ifndef AMR_NAV2_PLUGINS__FLEET_PEERS_HPP_
#define AMR_NAV2_PLUGINS__FLEET_PEERS_HPP_

// FleetPeerLayer의 ROS 없는 계산: 다른 로봇 위치(/fleet/robot_states, 2 Hz)를 지금 시각으로 외삽하고
// 차체 사각형을 만든다. LiDAR 스캔면(지면 0.38 m)이 차체(0.03~0.33 m) 위를 지나 로봇끼리 서로를
// 못 보므로 위치 공유로 대신한다.

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace amr_nav2_plugins
{

struct Point2
{
  double x;
  double y;
};

struct PeerPose
{
  double x;
  double y;
  double yaw;
};

class PeerTracker
{
public:
  struct Config
  {
    double timeout_s = 2.0;            // 이보다 오래 소식이 없으면 뺀다 (상태 주기 0.5 s의 4배)
    double max_extrapolation_s = 1.0;  // 외삽 상한. 그 뒤로는 마지막 속도로 더 밀지 않는다
    double max_speed = 2.0;            // 속도 추정 상한 (로봇 최대 2.0 m/s). 위치가 튀면 잘라 낸다
    double max_velocity_dt_s = 2.0;    // 두 상태 간격이 이보다 길면 속도를 추정하지 않는다
  };

  PeerTracker() = default;
  explicit PeerTracker(const Config & config)
  : config_(config) {}

  // 같은 로봇의 더 오래된(또는 같은 시각) 상태는 무시한다
  void update(const std::string & id, double t, double x, double y, double yaw)
  {
    auto it = peers_.find(id);
    if (it == peers_.end()) {
      peers_[id] = Peer{t, x, y, yaw, 0.0, 0.0};
      return;
    }
    Peer & p = it->second;
    const double dt = t - p.t;
    if (dt <= 0.0) {
      return;
    }
    double vx = 0.0;
    double vy = 0.0;
    if (dt <= config_.max_velocity_dt_s) {
      vx = (x - p.x) / dt;
      vy = (y - p.y) / dt;
      const double speed = std::hypot(vx, vy);
      if (speed > config_.max_speed) {
        vx *= config_.max_speed / speed;
        vy *= config_.max_speed / speed;
      }
    }
    p = Peer{t, x, y, yaw, vx, vy};
  }

  // now 기준 살아 있는 로봇들의 외삽 자세. 시계가 상태 시각보다 앞서 있지 않으면(age < 0) 외삽하지 않는다
  std::vector<PeerPose> predict(double now) const
  {
    std::vector<PeerPose> out;
    for (const auto & kv : peers_) {
      const Peer & p = kv.second;
      const double age = now - p.t;
      if (age > config_.timeout_s) {
        continue;
      }
      const double dt = std::clamp(age, 0.0, config_.max_extrapolation_s);
      out.push_back(PeerPose{p.x + p.vx * dt, p.y + p.vy * dt, p.yaw});
    }
    return out;
  }

  std::size_t size() const {return peers_.size();}

private:
  struct Peer
  {
    double t;
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
  };
  Config config_;
  std::map<std::string, Peer> peers_;
};

// 중심 (x, y), 방향 yaw인 사각형(반길이·반폭 + padding)의 꼭짓점 4개 (반시계)
inline std::vector<Point2> footprintCorners(
  const PeerPose & pose, double half_length, double half_width, double padding)
{
  const double hl = half_length + padding;
  const double hw = half_width + padding;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  std::vector<Point2> out;
  for (const auto & local : {Point2{hl, hw}, Point2{-hl, hw}, Point2{-hl, -hw}, Point2{hl, -hw}}) {
    out.push_back(Point2{pose.x + c * local.x - s * local.y, pose.y + s * local.x + c * local.y});
  }
  return out;
}

// 다각형 안이면 true (짝홀 규칙). 꼭짓점이 3개 미만이면 false
inline bool pointInPolygon(const std::vector<Point2> & polygon, double x, double y)
{
  bool inside = false;
  const std::size_t n = polygon.size();
  if (n < 3) {
    return false;
  }
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2 & a = polygon[i];
    const Point2 & b = polygon[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}

// 노드 네임스페이스("/amr_01" 또는 "/amr_01/sub")의 첫 칸 = 로봇 이름. 없으면 빈 문자열
inline std::string robotIdFromNamespace(const std::string & ns)
{
  const std::size_t start = ns.find_first_not_of('/');
  if (start == std::string::npos) {
    return "";
  }
  const std::size_t end = ns.find('/', start);
  return ns.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__FLEET_PEERS_HPP_

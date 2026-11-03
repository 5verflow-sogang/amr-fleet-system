// Gazebo 바퀴 joint_states(joint_states_raw, 물리 스텝마다 약 1000 Hz)를 rate_hz(기본 50)로 줄여 joint_states로 낸다.
//
// Fortress JointStatePublisher는 발행 주기 설정이 없어(<update_rate>를 넣어도 1000 Hz) 브리지 뒤에서 줄인다.
// 받는 쪽(wheel_odometry Python, robot_state_publisher)이 1000 Hz를 받으면 5대에서 약 2코어를 썼다.
// 1000 Hz를 받는 건 이 노드 하나라 C++로 둔다 (Python이면 받는 비용이 그대로 남는다).
// 바퀴 각도는 누적값이라 중간 메시지를 버려도 오도메트리 적분 결과는 같다 (amr_odometry encoder_model).
// 시각은 메시지 stamp(sim 시간) 기준이라 use_sim_time이 필요 없다. 규칙은 include/amr_gazebo/throttle.hpp.

#include <cstdint>
#include <memory>
#include <optional>

#include "amr_gazebo/throttle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateThrottle : public rclcpp::Node
{
public:
  JointStateThrottle()
  : Node("joint_state_throttle")
  {
    const double rate_hz = declare_parameter("rate_hz", 50.0);
    period_ns_ = static_cast<int64_t>(1e9 / rate_hz);
    pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::UniquePtr msg) {
        const int64_t now_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (amr_gazebo::should_publish(last_ns_, now_ns, period_ns_)) {
          last_ns_ = now_ns;
          pub_->publish(std::move(msg));
        }
      });
    RCLCPP_INFO(get_logger(), "joint_states_raw → joint_states %g Hz", rate_hz);
  }

private:
  int64_t period_ns_;
  std::optional<int64_t> last_ns_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateThrottle>());
  rclcpp::shutdown();
  return 0;
}

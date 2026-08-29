// arm_safe.cpp — "does it arm with ALL safety checks?" test for a real vehicle.
//
// Unlike arm_test.cpp (which force-arms and bypasses preflight), this sends a
// NORMAL arm command: PX4 only arms if every preflight/health check passes
// (GPS/EKF, sensors, etc.). If arming is denied, the vehicle simply stays
// disarmed and this node reports it — nothing spins.
//
// Sequence: ARM (normal), hold for `hold_s` seconds, then DISARM and exit.
//
// Run (single real drone at root namespace, MAV_SYS_ID 1):
//   ros2 run multi_sim arm_safe
// Override, e.g. longer hold or another sysid:
//   ros2 run multi_sim arm_safe --ros-args -p hold_s:=8.0 -p sysid:=1

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <cstdlib>
#include <string>

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;

class ArmSafe : public rclcpp::Node {
public:
  ArmSafe() : rclcpp::Node("arm_safe") {
    ns_     = this->declare_parameter<std::string>("ns", "");     // "" = root (/fmu/...)
    sysid_  = this->declare_parameter<int>("sysid", 1);           // must match MAV_SYS_ID
    hold_s_ = this->declare_parameter<double>("hold_s", 5.0);     // seconds armed before disarm

    cmd_pub_ = this->create_publisher<VehicleCommand>(ns_ + "/fmu/in/vehicle_command", 10);
    // PX4 1.16+ versions /fmu/out topics; default "_v1", set PX4_OUT_SUFFIX="" for older SITL.
    const char* sfx_env = std::getenv("PX4_OUT_SUFFIX");
    const std::string sfx = sfx_env ? sfx_env : "_v1";
    status_sub_ = this->create_subscription<VehicleStatus>(
        ns_ + "/fmu/out/vehicle_status" + sfx, rclcpp::SensorDataQoS(),
        [this](const VehicleStatus& s) { arming_state_ = s.arming_state; });

    start_ = this->now();
    timer_ = this->create_wall_timer(200ms, [this]() { tick(); });

    RCLCPP_WARN(this->get_logger(),
        "arm_safe starting: ns='%s' sysid=%d hold=%.1fs (NORMAL arm, preflight checks ON)",
        ns_.c_str(), sysid_, hold_s_);
  }

private:
  void tick() {
    const double t = (this->now() - start_).seconds();

    if (t < hold_s_) {
      send_arm(true);
      if (!armed_announced_) {
        RCLCPP_INFO(this->get_logger(), "-> sending ARM (normal, checks enforced)");
        armed_announced_ = true;
      }
      // If it hasn't armed a couple of seconds in, a preflight check is blocking it.
      if (t > 2.0 && arming_state_ != 2 && !denied_warned_) {
        RCLCPP_WARN(this->get_logger(),
            "still DISARMED after 2s -> a preflight check is likely blocking arming "
            "(check GPS/EKF, sensors). See QGC for the exact reason.");
        denied_warned_ = true;
      }
    } else if (t < hold_s_ + 1.0) {
      send_arm(false);
      if (!disarm_announced_) {
        RCLCPP_INFO(this->get_logger(), "-> sending DISARM");
        disarm_announced_ = true;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "arm_safe finished (last arming_state=%d; 2 = ARMED).",
                  static_cast<int>(arming_state_));
      rclcpp::shutdown();
      return;
    }

    if (static_cast<int>(t * 5) % 5 == 0) {
      RCLCPP_INFO(this->get_logger(), "t=%.1fs arming_state=%d (1=DISARMED, 2=ARMED)",
                  t, static_cast<int>(arming_state_));
    }
  }

  void send_arm(bool arm) {
    VehicleCommand v{};
    v.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    v.param1 = arm ? 1.0f : 0.0f;
    v.param2 = 0.0f;                 // 0 = normal (NOT the 21196 force magic number)
    v.target_system = static_cast<uint8_t>(sysid_);
    v.target_component = 1;
    v.source_system = 1;
    v.source_component = 1;
    v.from_external = true;
    v.timestamp = this->now().nanoseconds() / 1000;
    cmd_pub_->publish(v);
  }

  std::string ns_;
  int sysid_{1};
  double hold_s_{5.0};
  uint8_t arming_state_{0};
  bool armed_announced_{false}, disarm_announced_{false}, denied_warned_{false};
  rclcpp::Time start_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmSafe>());
  rclcpp::shutdown();
  return 0;
}

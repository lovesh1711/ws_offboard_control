// arm_test.cpp — minimal "does it arm?" test for a real vehicle.
//
// Sends only the ARM command (optionally force-armed to bypass preflight
// checks, e.g. indoors with no GPS/position). It does NOT use offboard or
// position setpoints, so it works without a position estimate.
//
// Sequence: force-ARM, hold for `hold_s` seconds, then DISARM and exit.
//
// !!! PROPELLERS OFF !!!  Force-arming skips all safety checks; the motors
// will spin as soon as the vehicle arms.
//
// Run (single real drone at root namespace, MAV_SYS_ID 1):
//   ros2 run multi_sim arm_test
// Override params, e.g. a longer hold or a namespaced/other-sysid vehicle:
//   ros2 run multi_sim arm_test --ros-args -p hold_s:=8.0 -p sysid:=1 -p ns:=""

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;

class ArmTest : public rclcpp::Node {
public:
  ArmTest() : rclcpp::Node("arm_test") {
    ns_     = this->declare_parameter<std::string>("ns", "");      // "" = root (/fmu/...)
    sysid_  = this->declare_parameter<int>("sysid", 1);            // must match MAV_SYS_ID
    force_  = this->declare_parameter<bool>("force", true);        // bypass preflight checks
    hold_s_ = this->declare_parameter<double>("hold_s", 5.0);      // seconds armed before disarm

    cmd_pub_ = this->create_publisher<VehicleCommand>(ns_ + "/fmu/in/vehicle_command", 10);
    status_sub_ = this->create_subscription<VehicleStatus>(
        ns_ + "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
        [this](const VehicleStatus& s) { arming_state_ = s.arming_state; });

    start_ = this->now();
    timer_ = this->create_wall_timer(200ms, [this]() { tick(); });

    RCLCPP_WARN(this->get_logger(),
        "arm_test starting: ns='%s' sysid=%d force=%d hold=%.1fs -- PROPS OFF!",
        ns_.c_str(), sysid_, static_cast<int>(force_), hold_s_);
  }

private:
  void tick() {
    const double t = (this->now() - start_).seconds();

    if (t < hold_s_) {
      send_arm(true);
      if (!armed_announced_) {
        RCLCPP_INFO(this->get_logger(), "-> sending ARM%s", force_ ? " (FORCE)" : "");
        armed_announced_ = true;
      }
    } else if (t < hold_s_ + 1.0) {
      send_arm(false);
      if (!disarm_announced_) {
        RCLCPP_INFO(this->get_logger(), "-> sending DISARM");
        disarm_announced_ = true;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "arm_test finished (last arming_state=%d; 2 = ARMED).",
                  static_cast<int>(arming_state_));
      rclcpp::shutdown();
      return;
    }

    // periodic feedback on the actual arming state reported by PX4
    if (static_cast<int>(t * 5) % 5 == 0) {
      RCLCPP_INFO(this->get_logger(), "t=%.1fs arming_state=%d (1=DISARMED, 2=ARMED)",
                  t, static_cast<int>(arming_state_));
    }
  }

  void send_arm(bool arm) {
    VehicleCommand v{};
    v.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    v.param1 = arm ? 1.0f : 0.0f;
    v.param2 = (arm && force_) ? 21196.0f : 0.0f;  // 21196 = PX4 "force" magic number
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
  bool force_{true};
  double hold_s_{5.0};
  uint8_t arming_state_{0};
  bool armed_announced_{false}, disarm_announced_{false};
  rclcpp::Time start_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTest>());
  rclcpp::shutdown();
  return 0;
}

// altitude_probe.cpp — diagnostic for EKF altitude accuracy in SITL.
//
// Question it answers: is the altitude estimate error a fixed noise floor, or does it
// grow during hover because accelerometer bias is weakly observable when the vehicle
// is not accelerating?
//
// Profile: arm -> climb to `alt` -> HOVER for `hover_s` -> fly a horizontal line back
// and forth (amplitude `amp`, period `period_s`) for `move_s` -> hold.
// Ground truth is sampled externally from Gazebo; this node only flies the profile.
//
//   ros2 run multi_sim altitude_probe --ros-args -p ns:=""       # instance 0
//   ros2 run multi_sim altitude_probe --ros-args -p ns:=/px4_1   # instance 1

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

class AltitudeProbe : public rclcpp::Node
{
public:
  AltitudeProbe() : Node("altitude_probe")
  {
    ns_       = declare_parameter<std::string>("ns", "");
    // PX4 instance N has MAV_SYS_ID N+1; a VehicleCommand addressed to the
    // wrong system is silently ignored, so this must match the instance.
    sysid_    = declare_parameter<int>("sysid", 1);
    alt_      = declare_parameter<double>("alt", 3.0);
    hover_s_  = declare_parameter<double>("hover_s", 60.0);
    move_s_   = declare_parameter<double>("move_s", 60.0);
    amp_      = declare_parameter<double>("amp", 4.0);
    period_s_ = declare_parameter<double>("period_s", 12.0);

    auto qos = rclcpp::SensorDataQoS();
    offb_pub_ = create_publisher<OffboardControlMode>(ns_ + "/fmu/in/offboard_control_mode", qos);
    traj_pub_ = create_publisher<TrajectorySetpoint>(ns_ + "/fmu/in/trajectory_setpoint", qos);
    cmd_pub_  = create_publisher<VehicleCommand>(ns_ + "/fmu/in/vehicle_command", 10);

    status_sub_ = create_subscription<VehicleStatus>(
        ns_ + "/fmu/out/vehicle_status", qos,
        [this](VehicleStatus::UniquePtr m) { armed_ = (m->arming_state == 2); });

    pos_sub_ = create_subscription<VehicleLocalPosition>(
        ns_ + "/fmu/out/vehicle_local_position", qos,
        [this](VehicleLocalPosition::UniquePtr m)
        { x0_valid_ ? void() : (x0_ = m->x, y0_ = m->y, x0_valid_ = true, void()); });

    timer_ = create_wall_timer(50ms, [this] { Tick(); });
    RCLCPP_INFO(get_logger(), "altitude_probe ns='%s' sysid=%d alt=%.1f hover=%.0fs move=%.0fs",
                ns_.c_str(), sysid_, alt_, hover_s_, move_s_);
  }

private:
  void Send(uint32_t cmd, float p1 = 0.f, float p2 = 0.f)
  {
    VehicleCommand m{};
    m.command = cmd; m.param1 = p1; m.param2 = p2;
    m.target_system = static_cast<uint8_t>(sysid_); m.target_component = 1;
    m.source_system = static_cast<uint8_t>(sysid_); m.source_component = 1;
    m.from_external = true;
    m.timestamp = get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(m);
  }

  void Tick()
  {
    const double t = (++ticks_) * 0.05;

    // Offboard requires a setpoint stream to already be running before the mode
    // switch is accepted, hence the 1 s of setpoints before commanding anything.
    OffboardControlMode m{};
    m.position = true;
    m.timestamp = get_clock()->now().nanoseconds() / 1000;
    offb_pub_->publish(m);

    double x = x0_, y = y0_;
    const char *phase = "climb";
    if (t > kArmAt + hover_s_)
    {
      const double tm = t - (kArmAt + hover_s_);
      if (tm < move_s_)
      {
        phase = "MOVING";
        x = x0_ + amp_ * std::sin(2.0 * M_PI * tm / period_s_);
      }
      else
      {
        phase = "hold";
      }
    }
    else if (t > kArmAt)
    {
      phase = "HOVER";
    }

    TrajectorySetpoint sp{};
    sp.position = {static_cast<float>(x), static_cast<float>(y),
                   static_cast<float>(-alt_)};
    sp.yaw = 0.0f;
    sp.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(sp);

    if (std::abs(t - kArmAt) < 0.03)
    {
      Send(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // OFFBOARD
      Send(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
      RCLCPP_INFO(get_logger(), "offboard + arm requested");
    }

    if (ticks_ % 100 == 0)
      RCLCPP_INFO(get_logger(), "t=%.0fs phase=%s armed=%d", t, phase, armed_ ? 1 : 0);
  }

  static constexpr double kArmAt = 1.0;

  std::string ns_;
  double alt_, hover_s_, move_s_, amp_, period_s_;
  int sysid_{1};
  double x0_{0.0}, y0_{0.0};
  bool x0_valid_{false}, armed_{false};
  uint64_t ticks_{0};

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offb_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AltitudeProbe>());
  rclcpp::shutdown();
  return 0;
}

// stack_test.cpp — hold two drones in a vertical stack and let the lower one sag.
//
// This is the Stage 4 gate for the downwash plugin. Unlike crossing_test, which gives a
// transient as the lower vehicle sweeps through the wake, this parks it in the wake and
// holds, so the steady-state altitude error is a clean number to compare against the
// physics library's prediction.
//
//   role=upper  hold at (spawn_x, spawn_y, alt)
//   role=lower  climb on its OWN column to (alt - gap), translate under the upper, hold
//
// The lower vehicle is told where the upper is as a DELTA (dn, de) in its own local
// frame, not as an absolute position. Each PX4 instance may or may not share a local
// origin with the others depending on how the estimator initialised, and a relative
// command is correct either way — the node captures its own spawn from
// vehicle_local_position and adds the delta. The shell script computes the delta from
// Gazebo ground truth, where both vehicles are certainly in one frame.
//
// PX4 NED: x north, y east, z down. `alt` and `gap` are positive up.
//
//   ros2 run multi_sim stack_test --ros-args -p role:=upper -p alt:=5.0
//   ros2 run multi_sim stack_test --ros-args -p role:=lower -p alt:=5.0 -p gap:=1.5 \
//        -p dn:=-3.0 -p de:=0.0 -p ns:=/px4_1 -p sysid:=2

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

class StackTest : public rclcpp::Node
{
public:
  StackTest() : Node("stack_test")
  {
    ns_        = declare_parameter<std::string>("ns", "");
    sysid_     = declare_parameter<int>("sysid", 1);
    role_      = declare_parameter<std::string>("role", "upper");
    alt_       = declare_parameter<double>("alt", 5.0);
    gap_       = declare_parameter<double>("gap", 1.5);
    dn_        = declare_parameter<double>("dn", 0.0);
    de_        = declare_parameter<double>("de", 0.0);
    climbS_    = declare_parameter<double>("climb_s", 20.0);
    approachS_ = declare_parameter<double>("approach_s", 20.0);

    holdAlt_ = (role_ == "lower") ? alt_ - gap_ : alt_;

    auto qos = rclcpp::SensorDataQoS();
    offb_pub_ = create_publisher<OffboardControlMode>(ns_ + "/fmu/in/offboard_control_mode", qos);
    traj_pub_ = create_publisher<TrajectorySetpoint>(ns_ + "/fmu/in/trajectory_setpoint", qos);
    cmd_pub_  = create_publisher<VehicleCommand>(ns_ + "/fmu/in/vehicle_command", 10);
    status_sub_ = create_subscription<VehicleStatus>(
        ns_ + "/fmu/out/vehicle_status", qos,
        [this](VehicleStatus::UniquePtr m) { armed_ = (m->arming_state == 2); });
    pos_sub_ = create_subscription<VehicleLocalPosition>(
        ns_ + "/fmu/out/vehicle_local_position", qos,
        [this](VehicleLocalPosition::UniquePtr m) { OnPosition(*m); });

    timer_ = create_wall_timer(50ms, [this] { Tick(); });
    RCLCPP_INFO(get_logger(),
        "stack_test role=%s sysid=%d hold_alt=%.2f gap=%.2f delta=(%.2f, %.2f)",
        role_.c_str(), sysid_, holdAlt_, gap_, dn_, de_);
  }

private:
  /// Latch the spawn column, but only from a position the estimator actually vouches
  /// for and only once it has stopped moving.
  ///
  /// Checking isfinite() is NOT enough, and getting this wrong is dangerous. Before the
  /// EKF has a position source it publishes a perfectly finite (0, 0) with xy_valid
  /// false. Latching that makes the vehicle believe it spawned at the world origin --
  /// which is exactly where the OTHER drone is -- and the "hold your own column" phase
  /// then flies it straight into the neighbour. That is a near-collision, not a
  /// cosmetic bug, and the separation monitor in experiment_stack.sh caught it at 8 cm.
  void OnPosition(const VehicleLocalPosition &_m)
  {
    if (!_m.xy_valid || !_m.z_valid)
      return;
    z_ = _m.z;
    vz_ = _m.vz;
    if (spawnValid_)
      return;

    // Require a run of samples that agree, so a single good message arriving mid-jump
    // during estimator initialisation cannot be latched.
    if (settle_ == 0 || std::hypot(_m.x - spawnX_, _m.y - spawnY_) > 0.15)
    {
      spawnX_ = _m.x;
      spawnY_ = _m.y;
      settle_ = 1;
      return;
    }
    if (++settle_ >= kSettleSamples)
    {
      spawnValid_ = true;
      RCLCPP_INFO(get_logger(), "spawn column latched at (%.2f, %.2f) after %d samples",
                  spawnX_, spawnY_, settle_);
    }
  }

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

    OffboardControlMode m{};
    m.position = true;
    m.timestamp = get_clock()->now().nanoseconds() / 1000;
    offb_pub_->publish(m);

    const double sx = spawnValid_ ? spawnX_ : 0.0;
    const double sy = spawnValid_ ? spawnY_ : 0.0;

    double x = sx, y = sy;
    const char *phase;

    if (role_ == "lower" && t >= climbS_)
    {
      // Ease under the upper vehicle. Both are already at their hold altitudes and are
      // separated vertically by `gap`, so the lateral transit is safe.
      const double f = std::clamp((t - climbS_) / approachS_, 0.0, 1.0);
      phase = (f < 1.0) ? "APPROACH" : "STACKED";
      x = sx + dn_ * f;
      y = sy + de_ * f;
      stacked_ = (f >= 1.0);
    }
    else
    {
      phase = "CLIMB";
    }

    TrajectorySetpoint sp{};
    sp.position = {static_cast<float>(x), static_cast<float>(y),
                   static_cast<float>(-holdAlt_)};
    sp.yaw = 0.0f;
    sp.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(sp);

    if (!armSent_ && spawnValid_ && t > 1.0)
    {
      armSent_ = true;
      Send(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // OFFBOARD
      Send(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
      RCLCPP_INFO(get_logger(), "offboard + arm requested (spawn %.2f, %.2f)", sx, sy);
    }

    // Altitude error is the whole measurement: the sag under downwash is the difference
    // between the commanded hold altitude and where the vehicle actually settles.
    if (ticks_ % 20 == 0)
    {
      RCLCPP_INFO(get_logger(),
          "t=%.1fs phase=%s armed=%d stacked=%d sp=(%.2f,%.2f,%.2f) z=%.3f err=%.3f vz=%.3f",
          t, phase, armed_ ? 1 : 0, stacked_ ? 1 : 0, x, y, -holdAlt_, -z_,
          -z_ - holdAlt_, vz_);
    }
  }

  /// Consecutive agreeing position samples required before the spawn is trusted.
  /// vehicle_local_position runs at ~30 Hz, so this is about a second.
  static constexpr int kSettleSamples = 40;

  std::string ns_, role_;
  int sysid_{1};
  double alt_, gap_, dn_, de_, climbS_, approachS_, holdAlt_;
  double spawnX_{0.0}, spawnY_{0.0};
  double z_{0.0}, vz_{0.0};
  int settle_{0};
  bool spawnValid_{false}, armed_{false}, armSent_{false}, stacked_{false};
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
  rclcpp::spin(std::make_shared<StackTest>());
  rclcpp::shutdown();
  return 0;
}

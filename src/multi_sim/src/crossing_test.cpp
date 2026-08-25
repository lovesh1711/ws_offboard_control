// crossing_test.cpp — one drone hovers, another flies a straight line beneath it.
//
// This is the canonical downwash encounter. Run it BEFORE the downwash plugin applies
// any force to establish the no-coupling baseline: the lower drone should fly a
// perfectly straight, level line. Run it again after the plugin is live and the
// difference is the downwash effect, with everything else held identical.
//
// PHASE SEQUENCING MATTERS. An earlier version sent the crosser straight to the far
// end of the line while both vehicles were still climbing; its path ran through the
// hoverer's column at ~2 m and they collided. So:
//
//   CLIMB    both hold their SPAWN xy and climb to their own altitude (3 m apart)
//   APPROACH crosser moves to the line start on ITS OWN side, hoverer already on station
//   TRAVERSE crosser flies the full line at constant speed, passing underneath once
//
// Geometry (PX4 NED: x north, y east, z down; `alt`/`gap` are positive up):
//   role=hover  hold at (spawn_x, spawn_y, alt)  -- nominally (0,0)
//   role=cross  traverse +half_len -> -half_len at (alt - gap), through the hoverer
//
//   ros2 run multi_sim crossing_test --ros-args -p role:=hover -p alt:=5.0
//   ros2 run multi_sim crossing_test --ros-args -p role:=cross -p alt:=5.0 \
//        -p gap:=1.0 -p speed:=1.0 -p ns:=/px4_1 -p sysid:=2

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

class CrossingTest : public rclcpp::Node
{
public:
  CrossingTest() : Node("crossing_test")
  {
    ns_      = declare_parameter<std::string>("ns", "");
    sysid_   = declare_parameter<int>("sysid", 1);
    role_    = declare_parameter<std::string>("role", "hover");
    alt_     = declare_parameter<double>("alt", 5.0);
    gap_     = declare_parameter<double>("gap", 1.0);
    speed_   = declare_parameter<double>("speed", 1.0);
    halfLen_ = declare_parameter<double>("half_len", 5.0);
    climbS_  = declare_parameter<double>("climb_s", 18.0);
    approachS_ = declare_parameter<double>("approach_s", 16.0);

    crossAlt_ = alt_ - gap_;
    traverse_ = 2.0 * halfLen_ / std::max(speed_, 1e-3);

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
        "crossing_test role=%s sysid=%d alt=%.1f gap=%.1f speed=%.2f half_len=%.1f",
        role_.c_str(), sysid_, alt_, gap_, speed_, halfLen_);
  }

private:
  /// Latch the spawn column, but only from a position the estimator vouches for and
  /// only once it has stopped moving.
  ///
  /// isfinite() is NOT a sufficient test, and this is the second half of the collision
  /// story in the header comment. Before the EKF has a position source it publishes a
  /// perfectly finite (0, 0) with xy_valid false; latching that convinces the vehicle it
  /// spawned at the world origin, which is where the OTHER drone is. Phase sequencing
  /// alone does not save you when the phases are computed from a wrong origin.
  void OnPosition(const VehicleLocalPosition &_m)
  {
    if (!_m.xy_valid || !_m.z_valid || spawnValid_)
      return;
    // Require a run of agreeing samples, so one good message arriving mid-jump during
    // estimator initialisation cannot be latched.
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
    m.velocity = true;          // velocity feed-forward keeps the constant-speed leg clean
    m.timestamp = get_clock()->now().nanoseconds() / 1000;
    offb_pub_->publish(m);

    // Everything is referenced to where this vehicle actually started, so the two
    // never contend for the same airspace during the climb.
    const double sx = spawnValid_ ? spawnX_ : 0.0;
    const double sy = spawnValid_ ? spawnY_ : 0.0;

    double x = sx, y = sy, z, vx = 0.0;
    const char *phase;

    if (role_ == "cross")
    {
      z = -crossAlt_;
      const double tApproach = climbS_;
      const double tStart    = climbS_ + approachS_;
      if (t < tApproach)
      {
        phase = "CLIMB";  x = sx;                       // straight up, no lateral motion
      }
      else if (t < tStart)
      {
        // Ease to the line start on our OWN side of the hoverer (+half_len), so the
        // approach never crosses the hoverer's column.
        const double f = std::clamp((t - tApproach) / approachS_, 0.0, 1.0);
        phase = "APPROACH";  x = sx + (halfLen_ - sx) * f;
        y = sy * (1.0 - f);                              // line up on y = 0
      }
      else
      {
        const double tt = t - tStart;
        if (tt < traverse_)
        { phase = "TRAVERSE"; x = halfLen_ - speed_ * tt; vx = -speed_; y = 0.0; }
        else
        { phase = "done";     x = -halfLen_;             y = 0.0; }
      }
    }
    else
    {
      z = -alt_;
      phase = (t < climbS_) ? "CLIMB" : "ON STATION";    // hold the spawn column
    }

    TrajectorySetpoint sp{};
    sp.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    sp.velocity = {static_cast<float>(vx), 0.0f, 0.0f};
    sp.yaw = 0.0f;
    sp.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(sp);

    // Arm only once the spawn position is known, so the first setpoint is never a
    // command to fly somewhere else.
    if (!armSent_ && spawnValid_ && t > kArmAt)
    {
      armSent_ = true;
      Send(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // OFFBOARD
      Send(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
      RCLCPP_INFO(get_logger(), "offboard + arm requested (spawn %.2f, %.2f)", sx, sy);
    }

    if (ticks_ % 100 == 0)
      RCLCPP_INFO(get_logger(), "t=%.0fs phase=%s armed=%d sp=(%.2f,%.2f,%.2f)",
                  t, phase, armed_ ? 1 : 0, x, y, z);
  }

  static constexpr double kArmAt = 1.0;
  /// Consecutive agreeing position samples required before the spawn is trusted.
  /// vehicle_local_position runs at ~30 Hz, so this is about a second.
  static constexpr int kSettleSamples = 40;

  std::string ns_, role_;
  int sysid_{1};
  double alt_, gap_, speed_, halfLen_, climbS_, approachS_, crossAlt_, traverse_;
  double spawnX_{0.0}, spawnY_{0.0};
  int settle_{0};
  bool spawnValid_{false}, armed_{false}, armSent_{false};
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
  rclcpp::spin(std::make_shared<CrossingTest>());
  rclcpp::shutdown();
  return 0;
}

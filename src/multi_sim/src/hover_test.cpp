// hover_test.cpp — simple OUTDOOR offboard hover test for a real vehicle.
//
// Sequence: stream offboard setpoints -> switch to OFFBOARD -> ARM (NORMAL,
// preflight checks enforced) -> ascend straight up to `alt` m over the takeoff
// spot -> hold `hold_s` s -> AUTO.LAND (which lands and disarms).
//
// Needs a valid position estimate (GPS/EKF) => use OUTDOORS.
// !!! Open area, props checked, people clear. Keep RC + KILL switch ready:
//     flipping to any manual mode (Position/Altitude/Stabilized) drops offboard
//     and hands control back to you.
//
// Run (root namespace, MAV_SYS_ID 1, defaults alt=3 m, hold=5 s):
//   ros2 run multi_sim hover_test
// Override:
//   ros2 run multi_sim hover_test --ros-args -p alt:=2.0 -p hold_s:=5.0 -p sysid:=1

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

class HoverTest : public rclcpp::Node {
public:
  HoverTest() : rclcpp::Node("hover_test") {
    ns_     = this->declare_parameter<std::string>("ns", "");    // "" = root (/fmu/...)
    sysid_  = this->declare_parameter<int>("sysid", 1);          // must match MAV_SYS_ID
    alt_    = this->declare_parameter<double>("alt", 3.0);       // metres above takeoff
    hold_s_ = this->declare_parameter<double>("hold_s", 5.0);    // seconds hovering

    offb_pub_ = create_publisher<OffboardControlMode>(ns_ + "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
    traj_pub_ = create_publisher<TrajectorySetpoint>(ns_ + "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
    cmd_pub_  = create_publisher<VehicleCommand>(ns_ + "/fmu/in/vehicle_command", 10);

    lpos_sub_ = create_subscription<VehicleLocalPosition>(
        ns_ + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        [this](const VehicleLocalPosition& m) {
          px_ = m.x; py_ = m.y; pz_ = m.z; heading_ = m.heading;
          pos_ok_ = m.xy_valid && m.z_valid;
        });
    status_sub_ = create_subscription<VehicleStatus>(
        ns_ + "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
        [this](const VehicleStatus& s) { armed_ = (s.arming_state == 2); });

    start_ = this->now();
    timer_ = create_wall_timer(100ms, [this] { tick(); });   // 10 Hz (offboard needs > 2 Hz)

    RCLCPP_WARN(get_logger(),
        "hover_test: alt=%.1f m, hold=%.1f s (NORMAL arm) -- OUTDOOR, CLEAR AREA, RC+KILL READY",
        alt_, hold_s_);
  }

private:
  enum class Stage { WARMUP, CLIMB, HOLD, LAND, DONE };

  void tick() {
    // Offboard heartbeat every tick (position control), always.
    publish_offboard_mode();

    // Need a position estimate before we can do anything meaningful.
    if (!pos_ok_) {
      publish_setpoint(0.f, 0.f, 0.f, 0.f);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "waiting for a valid position estimate (GPS/EKF)...");
      return;
    }

    switch (stage_) {
      case Stage::WARMUP: {
        if (!captured_) {                       // lock the takeoff spot + heading
          x0_ = px_; y0_ = py_; z0_ = pz_; yaw0_ = heading_; captured_ = true;
        }
        publish_setpoint(x0_, y0_, z0_, yaw0_); // hold on the ground while streaming
        ++ticks_;
        // After ~1 s of setpoints, (re)command OFFBOARD + ARM every ~0.5 s until armed.
        if (ticks_ >= 10 && !armed_ && ticks_ % 5 == 0) {
          publish_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);           // OFFBOARD
          publish_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);  // normal ARM
        }
        if (ticks_ == 40 && !armed_) {
          RCLCPP_WARN(get_logger(), "not armed after ~4 s -> preflight likely blocking (check GPS/EKF in QGC).");
        }
        if (armed_) {
          RCLCPP_INFO(get_logger(), "armed -> climbing to %.1f m", alt_);
          stage_ = Stage::CLIMB;
        }
        break;
      }
      case Stage::CLIMB: {
        publish_setpoint(x0_, y0_, z0_ - static_cast<float>(alt_), yaw0_);  // NED: up = smaller z
        const double cur_alt = z0_ - pz_;                                   // metres above takeoff
        if (std::fabs(cur_alt - alt_) < 0.3) {
          hover_start_ = this->now();
          RCLCPP_INFO(get_logger(), "reached %.1f m -> holding %.1f s", cur_alt, hold_s_);
          stage_ = Stage::HOLD;
        }
        break;
      }
      case Stage::HOLD: {
        publish_setpoint(x0_, y0_, z0_ - static_cast<float>(alt_), yaw0_);
        if ((this->now() - hover_start_).seconds() >= hold_s_) {
          RCLCPP_INFO(get_logger(), "hold done -> LANDING (AUTO.LAND)");
          stage_ = Stage::LAND;
        }
        break;
      }
      case Stage::LAND: {
        if (!land_sent_) {
          publish_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.f, 0.f);  // PX4 auto-lands + disarms
          land_sent_ = true;
          land_time_ = this->now();
        }
        // Keep holding the setpoint briefly; then finish once disarmed (landed) or after a timeout.
        publish_setpoint(x0_, y0_, pz_, yaw0_);
        const double since_land = (this->now() - land_time_).seconds();
        if ((!armed_ && since_land > 2.0) || since_land > 30.0) {
          RCLCPP_INFO(get_logger(), "landed & disarmed. hover_test finished.");
          stage_ = Stage::DONE;
          rclcpp::shutdown();
        }
        break;
      }
      case Stage::DONE:
      default:
        break;
    }
  }

  void publish_offboard_mode() {
    OffboardControlMode m{};
    m.position = true;
    m.timestamp = this->now().nanoseconds() / 1000;
    offb_pub_->publish(m);
  }

  void publish_setpoint(float x, float y, float z, float yaw) {
    TrajectorySetpoint sp{};
    sp.position = {x, y, z};
    sp.velocity = {NAN, NAN, NAN};
    sp.acceleration = {NAN, NAN, NAN};
    sp.jerk = {NAN, NAN, NAN};
    sp.yaw = yaw;
    sp.yawspeed = NAN;
    sp.timestamp = this->now().nanoseconds() / 1000;
    traj_pub_->publish(sp);
  }

  void publish_command(uint16_t command, float p1, float p2) {
    VehicleCommand v{};
    v.command = command;
    v.param1 = p1;
    v.param2 = p2;
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
  double alt_{3.0}, hold_s_{5.0};

  bool pos_ok_{false}, captured_{false}, armed_{false}, land_sent_{false};
  float px_{NAN}, py_{NAN}, pz_{NAN}, heading_{0.f};
  float x0_{0.f}, y0_{0.f}, z0_{0.f}, yaw0_{0.f};
  int ticks_{0};
  Stage stage_{Stage::WARMUP};
  rclcpp::Time start_, hover_start_, land_time_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offb_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoverTest>());
  rclcpp::shutdown();
  return 0;
}

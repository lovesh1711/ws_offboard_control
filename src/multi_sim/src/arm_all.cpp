// arm_all.cpp — simultaneous multi-drone ARM test using the SAME single-process,
// multi-threaded architecture as mission_setpoints:
//   * one process, one node,
//   * a MultiThreadedExecutor with a pool of worker threads,
//   * one mutually-exclusive callback group per drone (its timer + status sub),
//     so every drone is serviced concurrently on its own thread with no locks.
//
// For every drone in the fleet config, concurrently: ARM -> hold -> DISARM -> exit.
// This is a GROUND test — !!! PROPS OFF !!!  (force-arm spins the motors).
//
// Fleet config ($MISSION_CONFIG_FILE, same file as the mission — only the first
// two fields are used here):
//     ns sysid [wait tx ty alt ...]
// ns is the drone namespace ("/uav_0", "/uav_1", or "-"/"root" for the root ns).
//
// Run:
//   MISSION_CONFIG_FILE=/tmp/fleet.txt ros2 run multi_sim arm_all
//   ... --ros-args -p force:=false -p hold_s:=5.0    (normal arm / longer hold)

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;

struct Drone {
  std::string ns;
  int sysid{1};
  rclcpp::CallbackGroup::SharedPtr cb_group;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub;
  rclcpp::Time start;
  uint8_t arming_state{0};                 // only touched by this drone's callback group
  bool arm_announced{false}, disarm_announced{false}, done{false};
};

class ArmAll : public rclcpp::Node {
public:
  ArmAll() : rclcpp::Node("arm_all") {
    force_  = this->declare_parameter<bool>("force", true);     // bypass preflight (props off!)
    hold_s_ = this->declare_parameter<double>("hold_s", 5.0);   // seconds armed before disarm

    // PX4 1.16+ versions /fmu/out topics; default "_v1", set PX4_OUT_SUFFIX="" for older SITL.
    const char* sfx_env = std::getenv("PX4_OUT_SUFFIX");
    const std::string sfx = sfx_env ? sfx_env : "_v1";

    const auto cfgs = load_configs();
    if (cfgs.empty()) {
      throw std::runtime_error("no drones in fleet config ($MISSION_CONFIG_FILE)");
    }

    for (const auto& [ns, sysid] : cfgs) {
      auto d = std::make_shared<Drone>();
      d->ns = ns;
      d->sysid = sysid;

      // One mutually-exclusive callback group per drone: its timer and its status
      // subscription never run concurrently, so per-drone state needs no locks.
      d->cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      d->cmd_pub = create_publisher<VehicleCommand>(ns + "/fmu/in/vehicle_command", 10);

      rclcpp::SubscriptionOptions so;
      so.callback_group = d->cb_group;
      d->status_sub = create_subscription<VehicleStatus>(
          ns + "/fmu/out/vehicle_status" + sfx, rclcpp::SensorDataQoS(),
          [d](const VehicleStatus& s) { d->arming_state = s.arming_state; }, so);

      d->start = this->now();
      d->timer = create_wall_timer(200ms, [this, d]() { tick(*d); }, d->cb_group);

      drones_.push_back(d);
    }

    RCLCPP_WARN(get_logger(),
        "arm_all: %zu drone(s), force=%d hold=%.1fs -- PROPS OFF! "
        "(one process, MultiThreadedExecutor, one callback group per drone)",
        drones_.size(), static_cast<int>(force_), hold_s_);
  }

private:
  std::vector<std::pair<std::string, int>> load_configs() {
    std::vector<std::pair<std::string, int>> out;
    const char* env = std::getenv("MISSION_CONFIG_FILE");
    std::ifstream f(env ? env : "/tmp/mission_config.txt");
    std::string line;
    while (std::getline(f, line)) {
      const auto h = line.find('#');
      if (h != std::string::npos) line.erase(h);
      std::istringstream iss(line);
      std::string ns;
      int sysid;
      if (iss >> ns >> sysid) {
        if (ns == "/" || ns == "-" || ns == "root") ns.clear();   // root namespace
        out.emplace_back(ns, sysid);
      }
    }
    return out;
  }

  void tick(Drone& d) {
    if (d.done) return;
    const double t = (this->now() - d.start).seconds();

    if (t < hold_s_) {
      send_arm(d, true);
      if (!d.arm_announced) {
        RCLCPP_INFO(get_logger(), "[%s] -> ARM%s", nm(d).c_str(), force_ ? " (FORCE)" : "");
        d.arm_announced = true;
      }
    } else if (t < hold_s_ + 1.0) {
      send_arm(d, false);
      if (!d.disarm_announced) {
        RCLCPP_INFO(get_logger(), "[%s] -> DISARM (last arming_state=%d; 2=ARMED)",
                    nm(d).c_str(), static_cast<int>(d.arming_state));
        d.disarm_announced = true;
      }
    } else {
      RCLCPP_INFO(get_logger(), "[%s] done.", nm(d).c_str());
      d.done = true;
      maybe_shutdown();
    }
  }

  void send_arm(Drone& d, bool arm) {
    VehicleCommand v{};
    v.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    v.param1 = arm ? 1.0f : 0.0f;
    v.param2 = (arm && force_) ? 21196.0f : 0.0f;   // 21196 = PX4 "force" magic number
    v.target_system = static_cast<uint8_t>(d.sysid);
    v.target_component = 1;
    v.source_system = 1;
    v.source_component = 1;
    v.from_external = true;
    v.timestamp = this->now().nanoseconds() / 1000;
    d.cmd_pub->publish(v);
  }

  std::string nm(const Drone& d) const { return d.ns.empty() ? "root" : d.ns; }

  void maybe_shutdown() {
    for (const auto& d : drones_) {
      if (!d->done) return;
    }
    RCLCPP_INFO(get_logger(), "all drones finished. arm_all done.");
    rclcpp::shutdown();
  }

  bool force_{true};
  double hold_s_{5.0};
  std::vector<std::shared_ptr<Drone>> drones_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmAll>();
  rclcpp::executors::MultiThreadedExecutor exec;   // worker-thread pool -> drones in parallel
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

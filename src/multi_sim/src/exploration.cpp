


// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/executors/multi_threaded_executor.hpp>
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>

// #include <chrono>
// #include <cstdint>
// #include <memory>
// #include <string>
// #include <vector>
// #include <thread>
// #include <algorithm>
// #include <atomic>

// using namespace std::chrono_literals;
// using px4_msgs::msg::OffboardControlMode;
// using px4_msgs::msg::TrajectorySetpoint;
// using px4_msgs::msg::VehicleCommand;

// struct DroneIF {
//   std::string ns;
//   uint8_t mav_sys_id;
//   rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub;
//   rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub;
//   rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub;
// };

// class OffboardControlMultiMT : public rclcpp::Node {
// public:
//   OffboardControlMultiMT()
//   : rclcpp::Node("offboard_control_multi_mt")
//   {
//     // Configure as many drones as needed: namespace + MAV_SYS_ID (px4_instance + 1)
//     drones_.push_back(make_drone_if("/px4_1", 2));
//     drones_.push_back(make_drone_if("/px4_2", 3));  // add more entries for larger fleets
//     // drones_.push_back(make_drone_if("/px4_3", 4));

//     // One callback group and one 10 Hz timer per drone, enabling parallel execution
//     for (size_t i = 0; i < drones_.size(); ++i) {
//       auto grp = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//       groups_.push_back(grp);

//       // Per-drone counter for offboard warmup; shared_ptr to avoid copying atomics
//       auto counter = std::make_shared<std::atomic<uint64_t>>(0);
//       counters_.push_back(counter);

//       auto cb = [this, i, counter]() {
//         auto &d = drones_[i];
//         publish_offboard_control_mode(d);
//         publish_trajectory_setpoint(d);

//         auto cnt = counter->load();
//         if (cnt == 10) {
//           // Switch to OFFBOARD: param1=1 (custom), param2=6 (OFFBOARD)
//           publish_vehicle_command(d, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
//           // Arm
//           publish_vehicle_command(d, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
//           RCLCPP_INFO(this->get_logger(), "OFFBOARD+ARM sent to %s (SYSID=%u)", d.ns.c_str(), d.mav_sys_id);
//         }
//         if (cnt < 11) counter->store(cnt + 1);
//       };

//       // Bind timer to its callback group so the MultiThreadedExecutor can run them concurrently
//       auto timer = this->create_wall_timer(100ms, cb, grp);
//       timers_.push_back(timer);
//     }

//     RCLCPP_INFO(this->get_logger(),
//       "Started multi-threaded offboard controller for %zu drones", drones_.size());
//   }

// private:
//   DroneIF make_drone_if(const std::string &ns, uint8_t mav_sys_id) {
//     DroneIF d;
//     d.ns = ns;
//     d.mav_sys_id = mav_sys_id;
//     d.offboard_pub = this->create_publisher<OffboardControlMode>(ns + "/fmu/in/offboard_control_mode", 10);
//     d.traj_pub     = this->create_publisher<TrajectorySetpoint>(ns + "/fmu/in/trajectory_setpoint", 10);
//     d.cmd_pub      = this->create_publisher<VehicleCommand>(ns + "/fmu/in/vehicle_command", 10);
//     return d;
//   }

//   void publish_offboard_control_mode(DroneIF &d) {
//     OffboardControlMode msg{};
//     msg.position = true;
//     msg.velocity = false;
//     msg.acceleration = false;
//     msg.attitude = false;
//     msg.body_rate = false;
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     d.offboard_pub->publish(msg);
//   }

//   void publish_trajectory_setpoint(DroneIF &d) {
//     TrajectorySetpoint msg{};
//     msg.position = {0.0f, 0.0f, -5.0f};
//     msg.yaw = -3.14f;  // [-pi, pi]
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     d.traj_pub->publish(msg);
//   }

//   void publish_vehicle_command(DroneIF &d, uint16_t command, float p1 = 0.0f, float p2 = 0.0f) {
//     VehicleCommand v{};
//     v.param1 = p1;
//     v.param2 = p2;
//     v.command = command;
//     v.target_system = d.mav_sys_id;      // match receiving MAV_SYS_ID; or use 0
//     v.target_component = 1;
//     v.source_system = 1;
//     v.source_component = 1;
//     v.from_external = true;
//     v.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     d.cmd_pub->publish(v);
//   }

//   std::vector<DroneIF> drones_;
//   std::vector<rclcpp::CallbackGroup::SharedPtr> groups_;
//   std::vector<rclcpp::TimerBase::SharedPtr> timers_;
//   std::vector<std::shared_ptr<std::atomic<uint64_t>>> counters_;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<OffboardControlMultiMT>();

//   // Size the thread pool according to hardware and fleet size
//   size_t hw = std::max(1u, std::thread::hardware_concurrency());
//   size_t n_threads = std::max<size_t>(2, std::min<size_t>(hw, 8)); // example: 2..8 threads
//   rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), n_threads);
//   exec.add_node(node);
//   exec.spin();
//   rclcpp::shutdown();
//   return 0;
// }




#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstdlib>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

struct DroneConfig {
  std::string ns;
  uint8_t mav_sys_id;
  double wait_time_s;
  float target_x;
  float target_y;
  float cruise_alt_m;
};

enum class Stage {
  WAIT_START,
  WARMUP,
  OFFBOARD_ARMED,
  ASCEND,
  CRUISE,
  DESCEND_SITE,
  HOLD,
  ASCEND_BACK,
  CRUISE_BACK,
  LAND,
  DONE
};

struct DroneIF {
  DroneConfig cfg;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub;

  rclcpp::CallbackGroup::SharedPtr cb_group;
  rclcpp::TimerBase::SharedPtr timer;

  // State
  Stage stage{Stage::WAIT_START};
  bool offboard_sent{false};
  bool armed_sent{false};
  bool sent_land{false};
  bool armed{false};        // latest arming state reported by PX4 (vehicle_status)
  int warmup_count{0};

  // Position
  bool pos_valid_xy{false};
  bool pos_valid_z{false};
  float px{NAN}, py{NAN}, pz{NAN};

  // Targets
  float tx{0.f}, ty{0.f}, tz{0.f};

  // Rate-limited "carrot" setpoint (what we actually command) + speed limits.
  // Lower horiz_speed/vert_speed to fly slower; the carrot is kept on a short
  // leash from the vehicle so PX4 never chases a far setpoint and overshoots.
  float cx{NAN}, cy{NAN}, cz{NAN};
  float horiz_speed{3.0f};   // m/s horizontal cruise
  float vert_speed{2.0f};    // m/s climb/descent
  float lead_max{4.0f};      // max distance the carrot may lead the vehicle (m)

  // Debounce
  bool reached_once{false};

  // Timing
  rclcpp::Time wait_start_wall;
  rclcpp::Time hold_start_wall;
  rclcpp::Time last_log_wall;   // for throttled per-drone status logging

  // Thresholds (loosened so a drone settling near the waypoint still advances)
  float xy_thresh{1.5f};
  float z_thresh{0.7f};
};

class OffboardControlMultiMT : public rclcpp::Node {
public:
  OffboardControlMultiMT()
  : rclcpp::Node("offboard_control_multi_mt")
  {
    // Configure each drone: ns, MAV_SYS_ID, wait_time_s, target_x, target_y, cruise_alt_m
    // Loaded from the file named in $MISSION_CONFIG_FILE (written by run_mission.sh),
    // with a built-in 2-drone default if no config file is present.
    std::vector<DroneConfig> cfgs = load_configs();

    // PX4 1.16+ versions the /fmu/out topics (e.g. vehicle_local_position_v1).
    // Default to "_v1" (matches PX4 1.17 hardware); override with
    // PX4_OUT_SUFFIX="" for older/unversioned SITL.
    const char* sfx_env = std::getenv("PX4_OUT_SUFFIX");
    const std::string sfx = sfx_env ? sfx_env : "_v1";

    drones_.reserve(cfgs.size());
    for (const auto& cfg : cfgs) {
      drones_.emplace_back(std::make_shared<DroneIF>());
      auto d = drones_.back();
      d->cfg = cfg;

      d->offboard_pub = this->create_publisher<OffboardControlMode>(
          cfg.ns + "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
      d->traj_pub = this->create_publisher<TrajectorySetpoint>(
          cfg.ns + "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
      d->cmd_pub = this->create_publisher<VehicleCommand>(
          cfg.ns + "/fmu/in/vehicle_command", 10);

      d->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      d->lpos_sub = this->create_subscription<VehicleLocalPosition>(
        
          cfg.ns + "/fmu/out/vehicle_local_position" + sfx,
          rclcpp::SensorDataQoS(),
          [d](const VehicleLocalPosition& msg) {
            d->px = msg.x;
            d->py = msg.y;
            d->pz = msg.z;
            d->pos_valid_xy = std::isfinite(msg.x) && std::isfinite(msg.y);
            d->pos_valid_z  = std::isfinite(msg.z);
            // RCLCPP_DEBUG(this->get_logger(), "[%s] pos_valid_xy=%d pos_valid_z=%d px=%.2f py=%.2f pz=%.2f",
            //  d->cfg.ns.c_str(), (int)d->pos_valid_xy, (int)d->pos_valid_z, msg.x, msg.y, msg.z);
          });

      d->status_sub = this->create_subscription<VehicleStatus>(
          cfg.ns + "/fmu/out/vehicle_status" + sfx,
          rclcpp::SensorDataQoS(),
          [d](const VehicleStatus& msg) {
            d->armed = (msg.arming_state == 2);  // 2 = ARMING_STATE_ARMED
          });
    //   d->lpos_sub = this->create_subscription<VehicleLocalPosition>(
    // cfg.ns + "/fmu/out/vehicle_local_position",
    // rclcpp::SensorDataQoS(),
    // [this, d](const VehicleLocalPosition::SharedPtr msg) {
    //   d->px = msg->x;
    //   d->py = msg->y;
    //   d->pz = msg->z;
    //   d->pos_valid_xy = std::isfinite(msg->x) && std::isfinite(msg->y);
    //   d->pos_valid_z  = std::isfinite(msg->z);
    //   RCLCPP_DEBUG(this->get_logger(), "[%s] pos=(%.2f,%.2f,%.2f) valid_xy=%d valid_z=%d",
    //                d->cfg.ns.c_str(), d->px, d->py, d->pz,
    //                (int)d->pos_valid_xy, (int)d->pos_valid_z);
    // });


      d->wait_start_wall = this->get_clock()->now();
      d->last_log_wall   = d->wait_start_wall;  // init from node clock so the
                                                // throttled log's time subtraction
                                                // uses a matching clock source

      auto cb = [this, d]() { this->per_drone_step(*d); };
      d->timer = this->create_wall_timer(20ms, cb, d->cb_group);
    }

    RCLCPP_INFO(this->get_logger(), "Started multi-threaded offboard controller for %zu drones", drones_.size());
  }

private:
  // Read drone configs from $MISSION_CONFIG_FILE (default /tmp/mission_config.txt).
  // One drone per line, whitespace-separated, '#' starts a comment:
  //   ns  sysid  wait_time_s  target_x  target_y  cruise_alt_m
  // Targets are in each drone's LOCAL NED frame (i.e. relative to its spawn point).
  std::vector<DroneConfig> load_configs() {
    std::vector<DroneConfig> cfgs;
    const char* env = std::getenv("MISSION_CONFIG_FILE");
    const std::string path = env ? env : "/tmp/mission_config.txt";

    std::ifstream f(path);
    if (f.is_open()) {
      std::string line;
      while (std::getline(f, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream iss(line);
        DroneConfig c;
        int sysid; double wait; float tx, ty, alt;
        if (iss >> c.ns >> sysid >> wait >> tx >> ty >> alt) {
          // A real single drone publishes at the root (/fmu/...), not under /px4_N.
          // Accept "/", "-", or "root" in the config to mean the empty (root) namespace.
          if (c.ns == "/" || c.ns == "-" || c.ns == "root") c.ns.clear();
          c.mav_sys_id  = static_cast<uint8_t>(sysid);
          c.wait_time_s = wait;
          c.target_x    = tx;
          c.target_y    = ty;
          c.cruise_alt_m = alt;
          cfgs.push_back(c);
        }
      }
    }

    if (cfgs.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "No usable config at '%s'; falling back to built-in 2-drone default", path.c_str());
      cfgs = {
        {"/px4_1", 2, 0.0, 10.0f, 10.0f, 5.0f},
        {"/px4_2", 3, 2.0, -10.0f, 10.0f, 8.0f}
      };
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu drone(s) from '%s'", cfgs.size(), path.c_str());
    }
    return cfgs;
  }

  void per_drone_step(DroneIF& d) {
    const auto now = this->get_clock()->now();

    // Wait gate
    if (d.stage == Stage::WAIT_START) {
      const double elapsed = (now - d.wait_start_wall).seconds();
      if (elapsed < d.cfg.wait_time_s) {
        return;
      }
      d.stage = Stage::WARMUP;
      d.warmup_count = 0;
      // initialize first target to home at cruise altitude
      d.tx = 0.0f; d.ty = 0.0f; d.tz = -d.cfg.cruise_alt_m;
    }

    // Always publish OffboardControlMode at control rate
    publish_offboard_mode(d, true, false, false);

    if (d.stage == Stage::WARMUP) {
      publish_setpoint(d, d.tx, d.ty, d.tz);
      ++d.warmup_count;
      // After a short setpoint warm-up, (re)command OFFBOARD + ARM every 10 ticks
      // (~0.2 s) until PX4 actually reports armed. Retrying makes the fleet robust
      // to transient arming-check failures right after spawn (which otherwise leave
      // some drones grounded when the command is sent only once).
      if (d.warmup_count >= 20 && !d.armed && d.warmup_count % 10 == 0) {
        publish_vehicle_command(d, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        publish_vehicle_command(d, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
      }
      if (d.armed) {
        // Armed and in OFFBOARD -> begin the mission
        d.stage = Stage::ASCEND;
        d.reached_once = false;
        d.tx = 0.0f; d.ty = 0.0f; d.tz = -d.cfg.cruise_alt_m;
        d.cx = d.cy = d.cz = NAN;   // reseed carrot from the current position
      }
      return;
    }
    

    // Mission FSM with local position checks
    if (!(d.pos_valid_xy && d.pos_valid_z)) {
      publish_setpoint(d, d.tx, d.ty, d.tz);
      return;
    }
    
    // const float dx = d.tx - d.px;
    // const float dy = d.ty - d.py;
    // const float dz = d.tz - d.pz;
    // const float dist_xy = std::hypot(dx, dy);
    // const float dist_z  = std::fabs(dz);

    // float xy_thresh{0.5f};
    // float z_thresh{0.2f};
    // bool reached_once{false};


    const float dx = d.tx - d.px;
    const float dy = d.ty - d.py;
    const float dz = d.tz - d.pz;

    
    // RCLCPP_INFO(this->get_logger(), " pos=(%.2f,%.2f,%.2f)",  d.px, d.py, d.pz);
    // RCLCPP_INFO(this->get_logger(), "[%s] stage=%d pos=(%.2f,%.2f,%.2f) tgt=(%.2f,%.2f,%.2f) dxdy=(%.2f,%.2f) dz=%.2f dist_xy=%.2f dist_z=%.2f",
    //         d.cfg.ns.c_str(), static_cast<int>(d.stage),
    //         d.px, d.py, d.pz, d.tx, d.ty, d.tz, dx, dy, dz, dist_xy, dist_z);

    const float dist_xy = std::hypot(dx, dy);
    const float dist_z  = std::fabs(dz);

    // Per-drone throttled status log (~every 2 s) so a stuck stage is diagnosable.
    if ((now - d.last_log_wall).seconds() >= 2.0) {
      d.last_log_wall = now;
      RCLCPP_INFO(this->get_logger(),
        "[%s] stage=%d pos=(%.1f,%.1f,%.1f) tgt=(%.1f,%.1f,%.1f) dxy=%.2f dz=%.2f valid_xy=%d valid_z=%d",
        d.cfg.ns.c_str(), static_cast<int>(d.stage),
        d.px, d.py, d.pz, d.tx, d.ty, d.tz, dist_xy, dist_z,
        static_cast<int>(d.pos_valid_xy), static_cast<int>(d.pos_valid_z));
    }

    // auto reached = [&](bool xy, bool z) {
    //   const bool ok = (!xy || dist_xy < d.xy_thresh) && (!z || dist_z < d.z_thresh);
    //   if (!ok) { d.reached_once = false; return false; }
    //   if (!d.reached_once) { d.reached_once = true; return false; }
    //   d.reached_once = false;
    //   return true;
    // };
    // auto reached = [&](bool need_xy, bool need_z) {
    //   const bool ok_xy = !need_xy || (dist_xy < d.xy_thresh);
    //   const bool ok_z  = !need_z  || (dist_z  < d.z_thresh);
    //   return ok_xy && ok_z;
    // };


    auto reached = [&](bool need_xy, bool need_z) {
      const bool ok_xy = !need_xy || (dist_xy < d.xy_thresh);
      const bool ok_z  = !need_z  || (dist_z  < d.z_thresh);
      const bool ok = ok_xy && ok_z;
      if (!ok) { d.reached_once = false; return false; }
      if (!d.reached_once) { d.reached_once = true; return false; }
      d.reached_once = false;
      return true;
    };


    

    switch (d.stage) {
      case Stage::ASCEND: {
        d.tx = 0.0f; d.ty = 0.0f; d.tz = -d.cfg.cruise_alt_m;
        
        
        
        if (reached(false, true)) {
          d.stage = Stage::CRUISE;
          d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -d.cfg.cruise_alt_m;
          if (d.stage==Stage::CRUISE){
          // RCLCPP_INFO(this->get_logger(), "Started multi-threaded offboard controller for %zu drones", drones_.size());
        }
          // advance_to(Stage::CRUISE, d.cfg.target_x, d.cfg.target_y, -d.cfg.cruise_alt_m);
        }
        break;
      }
      case Stage::CRUISE: {
        // RCLCPP_INFO(this->get_logger(), "cruising in action");
        d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -d.cfg.cruise_alt_m;
        if (reached(true, true)) {
          d.stage = Stage::DESCEND_SITE;
          d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -3.0f;
          // advance_to(Stage::DESCEND_SITE, d.cfg.target_x, d.cfg.target_y, -3.0f);
        }
        break;
      }
      case Stage::DESCEND_SITE: {
        d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -3.0f;
        if (reached(true, true)) {
          d.stage = Stage::HOLD;
          d.hold_start_wall = now;
        }
        break;
      }
      case Stage::HOLD: {
        d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -3.0f;
        if ((now - d.hold_start_wall).seconds() >= 2.0) {
          d.stage = Stage::ASCEND_BACK;
          d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -d.cfg.cruise_alt_m;
          // advance_to(Stage::ASCEND_BACK, d.cfg.target_x, d.cfg.target_y, -d.cfg.cruise_alt_m);
        }
        break;
      }
      case Stage::ASCEND_BACK: {
        d.tx = d.cfg.target_x; d.ty = d.cfg.target_y; d.tz = -d.cfg.cruise_alt_m;
        if (reached(false, true)) {
          d.stage = Stage::CRUISE_BACK;
          d.tx = 0.0f; d.ty = 0.0f; d.tz = -d.cfg.cruise_alt_m;
          // advance_to(Stage::CRUISE_BACK, 0.0f, 0.0f, -d.cfg.cruise_alt_m);
        }
        break;
      }
      case Stage::CRUISE_BACK: {
        d.tx = 0.0f; d.ty = 0.0f; d.tz = -d.cfg.cruise_alt_m;
        if (reached(true, true)) {
          d.stage = Stage::LAND;
        }
        break;
      }
      case Stage::LAND: {
        // Descend in OFFBOARD while holding home (0,0) so the carrot actively nulls
        // any horizontal drift, and only hand off to PX4's auto-land for the final
        // ~1 m. This lands right on the takeoff point instead of drifting during a
        // long auto-descent from cruise altitude. (pz is NED: 0 = ground, up = negative.)
        if (d.pz > -1.0f) {
          if (!d.sent_land) {
            publish_vehicle_command(d, VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.f, 0.f);
            d.sent_land = true;
          }
          d.tx = d.px; d.ty = d.py; d.tz = d.pz;   // hold in place for touchdown
        } else {
          d.tx = 0.0f; d.ty = 0.0f; d.tz = -0.8f;   // hold home, descend down to 1 m
        }
        break;
      }
      case Stage::DONE: default:
        break;
    }

    publish_flight_setpoint(d);
  }

  // Advance a rate-limited "carrot" toward (tx,ty,tz) and command that instead of
  // the far final target. Keeping the carrot near the vehicle (on a short leash)
  // bounds the flight speed and eliminates overshoot at the waypoint.
  void publish_flight_setpoint(DroneIF& d) {
    const float dt = 0.02f;   // 50 Hz control loop

    // Seed the carrot at the current position the first time (needs valid pos).
    if (!(std::isfinite(d.cx) && std::isfinite(d.cy) && std::isfinite(d.cz))) {
      if (d.pos_valid_xy && d.pos_valid_z) { d.cx = d.px; d.cy = d.py; d.cz = d.pz; }
      else { publish_setpoint(d, d.tx, d.ty, d.tz); return; }
    }

    const bool leash = d.pos_valid_xy && d.pos_valid_z;

    // Horizontal: step toward target as a 2-D vector (so diagonals aren't faster).
    const float ex = d.tx - d.cx, ey = d.ty - d.cy;
    const float eh = std::hypot(ex, ey);
    const float stepH = d.horiz_speed * dt;
    if (!leash || std::hypot(d.cx - d.px, d.cy - d.py) < d.lead_max) {
      if (eh <= stepH || eh < 1e-3f) { d.cx = d.tx; d.cy = d.ty; }
      else { d.cx += ex / eh * stepH; d.cy += ey / eh * stepH; }
    }

    // Vertical.
    const float ez = d.tz - d.cz;
    const float stepV = d.vert_speed * dt;
    if (!leash || std::fabs(d.cz - d.pz) < d.lead_max) {
      if (std::fabs(ez) <= stepV) d.cz = d.tz;
      else d.cz += (ez > 0.f ? stepV : -stepV);
    }

    publish_setpoint(d, d.cx, d.cy, d.cz);
  }

  void publish_offboard_mode(DroneIF& d, bool position, bool velocity, bool acceleration) {
    OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = position;
    msg.velocity = velocity;
    msg.acceleration = acceleration;
    msg.attitude = false;
    msg.body_rate = false;
    d.offboard_pub->publish(msg);
  }

  void publish_setpoint(DroneIF& d, float x, float y, float z) {
    TrajectorySetpoint sp{};
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    sp.position[0] = x;
    sp.position[1] = y;
    sp.position[2] = z; // NED: negative = up
    sp.velocity[0]=NAN;
    sp.velocity[1]=NAN;
    sp.velocity[2]=NAN;
    sp.jerk[0] = NAN;
    sp.jerk[1] = NAN;
    sp.jerk[2] = NAN;
    sp.yaw = 0.0f;
    sp.yawspeed=NAN;
    d.traj_pub->publish(sp);
  }

  void publish_vehicle_command(DroneIF& d, uint16_t command, float p1 = 0.f, float p2 = 0.f, float p7 = 0.f) {
    VehicleCommand v{};
    v.param1 = p1;
    v.param2 = p2;
    v.param7 = p7;
    v.command = command;
    v.target_system = d.cfg.mav_sys_id;
    v.target_component = 1;
    v.source_system = 1;
    v.source_component = 1;
    v.from_external = true;
    v.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    d.cmd_pub->publish(v);
  }

  std::vector<std::shared_ptr<DroneIF>> drones_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardControlMultiMT>();

  const size_t hw = std::max<size_t>(1, static_cast<size_t>(std::thread::hardware_concurrency()));
  const size_t n_threads = std::max<size_t>(2, std::min(hw, static_cast<size_t>(8)));
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), n_threads);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

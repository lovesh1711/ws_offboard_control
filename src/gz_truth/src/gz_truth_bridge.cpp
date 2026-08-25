// gz_truth_bridge.cpp — feed Gazebo ground-truth pose to PX4 as vision odometry.
//
// WHY THIS EXISTS
// PX4's simulated GPS injects hard-coded noise (0.5 m sigma on altitude, 0.2 m
// horizontal, in sensor_gps_sim). The resulting EKF altitude error is +-0.3..0.5 m and
// is NOT fixed by flight profile -- measured at 0.42 m peak in hover and 0.50 m while
// flying a line. That is too coarse for the downwash experiments, which need a known
// vertical separation of at least 2.5*l = 1.25 m to stay out of the near field.
//
// This bridge publishes Gazebo's exact pose on /fmu/in/vehicle_visual_odometry, which
// PX4 fuses as an external vision source. It is the SITL analogue of motion capture,
// and mirrors the RTK/mocap setup the hardware stage will need anyway.
//
// SIMULATION ONLY. Nothing here runs on a real vehicle.
//
//   ros2 run gz_truth gz_truth_bridge --ros-args -p n_vehicles:=2
//
// Companion PX4 parameters (see scripts/use_vision.sh):
//   EKF2_EV_CTRL  3   fuse external-vision horizontal + vertical position
//   EKF2_HGT_REF  3   use vision as the primary height reference
//   EKF2_GPS_CTRL 0   stop fusing GPS, so its noise cannot leak back in

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

using px4_msgs::msg::VehicleOdometry;

class GzTruthBridge : public rclcpp::Node
{
public:
  GzTruthBridge() : rclcpp::Node("gz_truth_bridge")
  {
    world_        = declare_parameter<std::string>("world", "default");
    modelPrefix_  = declare_parameter<std::string>("model_prefix", "x500");
    nVehicles_    = declare_parameter<int>("n_vehicles", 2);
    publishHz_    = declare_parameter<double>("publish_hz", 50.0);

    // PX4 instance 0 lives at the root namespace, instance N at /px4_N.
    for (int i = 0; i < nVehicles_; ++i)
    {
      const std::string ns = (i == 0) ? "" : ("/px4_" + std::to_string(i));
      pubs_.push_back(create_publisher<VehicleOdometry>(
          ns + "/fmu/in/vehicle_visual_odometry", rclcpp::SensorDataQoS()));
      RCLCPP_INFO(get_logger(), "%s_%d -> %s/fmu/in/vehicle_visual_odometry",
                  modelPrefix_.c_str(), i, ns.empty() ? "(root)" : ns.c_str());
    }

    const std::string topic = "/world/" + world_ + "/pose/info";
    if (!gzNode_.Subscribe(topic, &GzTruthBridge::OnPoses, this))
      RCLCPP_FATAL(get_logger(), "failed to subscribe to %s", topic.c_str());
    else
      RCLCPP_INFO(get_logger(), "subscribed to %s", topic.c_str());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publishHz_)),
        [this] { Publish(); });
  }

private:
  struct Sample { double x{}, y{}, z{}; bool valid{false}; };

  /// \brief Gazebo publishes every entity pose here; keep only the vehicle bases.
  void OnPoses(const gz::msgs::Pose_V &_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < _msg.pose_size(); ++i)
    {
      const auto &pose = _msg.pose(i);
      const std::string &name = pose.name();
      if (name.rfind(modelPrefix_ + "_", 0) != 0)
        continue;
      // Model-level poses only: link poses share the prefix but carry a '::'.
      if (name.find("::") != std::string::npos)
        continue;

      const int idx = ParseIndex(name);
      if (idx < 0 || idx >= nVehicles_)
        continue;

      Sample s;
      s.x = pose.position().x();
      s.y = pose.position().y();
      s.z = pose.position().z();
      s.valid = true;
      latest_[idx] = s;
    }
  }

  /// \brief "x500_3" -> 3, or -1 when the suffix is not a plain integer.
  int ParseIndex(const std::string &_name) const
  {
    const std::size_t pos = _name.rfind('_');
    if (pos == std::string::npos || pos + 1 >= _name.size())
      return -1;
    const std::string suffix = _name.substr(pos + 1);
    for (const char c : suffix)
      if (!std::isdigit(static_cast<unsigned char>(c)))
        return -1;
    return std::stoi(suffix);
  }

  void Publish()
  {
    std::unordered_map<int, Sample> snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = latest_;
    }

    const uint64_t now = get_clock()->now().nanoseconds() / 1000;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (const auto &[idx, s] : snapshot)
    {
      if (!s.valid || idx < 0 || idx >= static_cast<int>(pubs_.size()))
        continue;

      VehicleOdometry m{};
      m.timestamp = now;
      m.timestamp_sample = now;
      m.pose_frame = VehicleOdometry::POSE_FRAME_NED;

      // Gazebo world frame is ENU; PX4 wants NED.
      m.position = {static_cast<float>(s.y),
                    static_cast<float>(s.x),
                    static_cast<float>(-s.z)};

      // Position-only fusion (EKF2_EV_CTRL=3). Orientation and velocity are marked
      // invalid so the EKF keeps using the magnetometer for yaw and its own
      // integration for velocity, rather than expecting fields we do not supply.
      m.q = {nan, nan, nan, nan};
      m.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
      m.velocity = {nan, nan, nan};
      m.angular_velocity = {nan, nan, nan};

      // Ground truth, so the variance is nominal rather than measured. With
      // EKF2_EV_NOISE_MD=0 PX4 uses EKF2_EVP_NOISE instead of these anyway.
      m.position_variance = {1e-4f, 1e-4f, 1e-4f};
      m.orientation_variance = {nan, nan, nan};
      m.velocity_variance = {nan, nan, nan};
      m.quality = 100;

      pubs_[idx]->publish(m);
    }

    if (++ticks_ % static_cast<uint64_t>(publishHz_ * 10) == 0)
      RCLCPP_INFO(get_logger(), "streaming truth for %zu vehicle(s)", snapshot.size());
  }

  std::string world_, modelPrefix_;
  int nVehicles_{2};
  double publishHz_{50.0};
  uint64_t ticks_{0};

  gz::transport::Node gzNode_;
  std::mutex mutex_;
  std::unordered_map<int, Sample> latest_;
  std::vector<rclcpp::Publisher<VehicleOdometry>::SharedPtr> pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzTruthBridge>());
  rclcpp::shutdown();
  return 0;
}

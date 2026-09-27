// ROS 2 wrapper around the estimator core.
//
// Subscribes:  /vehicle/front_bogie_velocity, /vehicle/rear_bogie_velocity (VelocitySensor)
//              /vehicle/driver_position_cmd (DriverControllerCommand, if the type is available)
//              /sensing/gnss/{master,rover}/fix (NavSatFix, initial alignment only by default)
// Publishes:   /result/velocity (VelocitySensor), /result/position (nav_msgs/Odometry),
//              /result/slip_status (std_msgs/String, compact JSON), /diagnostics
//
// Single-threaded executor: all callbacks run sequentially -> no data races.
// Output stamps are data (bag) time of the newest processed input, as required by the checker.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "tram_vehicle_msgs/msg/velocity_sensor.hpp"
#if __has_include("tram_vehicle_msgs/msg/driver_controller_command.hpp")
#include "tram_vehicle_msgs/msg/driver_controller_command.hpp"
#define TRAM_HAVE_DRIVER_CMD 1
#else
#define TRAM_HAVE_DRIVER_CMD 0
#endif

#include "tram_odometry/core/estimator.hpp"
#include "tram_odometry/core/params_table.hpp"
#include "tram_odometry/core/route_map.hpp"

namespace tram_odometry
{

using VelocitySensor = tram_vehicle_msgs::msg::VelocitySensor;
using SteadyClock = std::chrono::steady_clock;

class TramOdometryNode : public rclcpp::Node
{
public:
  explicit TramOdometryNode(const rclcpp::NodeOptions & options)
  : Node("tram_odometry_node", options)
  {
    // ---------------- parameters (core table + node specific)
    EstimatorParams ep; TractionParams tp; GeoParams gp;
    for (const auto & p : make_param_table(ep, tp, gp)) {
      switch (p.kind) {
        case ParamRef::Kind::Double: {
            double * v = static_cast<double *>(p.ptr);
            *v = declare_parameter<double>(p.name, *v);
            break;
          }
        case ParamRef::Kind::Int: {
            int * v = static_cast<int *>(p.ptr);
            *v = static_cast<int>(declare_parameter<int64_t>(p.name, *v));
            break;
          }
        case ParamRef::Kind::Bool: {
            bool * v = static_cast<bool *>(p.ptr);
            *v = declare_parameter<bool>(p.name, *v);
            break;
          }
        case ParamRef::Kind::String: {
            std::string * v = static_cast<std::string *>(p.ptr);
            *v = declare_parameter<std::string>(p.name, *v);
            break;
          }
      }
    }
    const auto t_front = declare_parameter<std::string>("topics.front", "/vehicle/front_bogie_velocity");
    const auto t_rear = declare_parameter<std::string>("topics.rear", "/vehicle/rear_bogie_velocity");
    const auto t_cmd = declare_parameter<std::string>("topics.command", "/vehicle/driver_position_cmd");
    const auto t_mfix = declare_parameter<std::string>("topics.gnss_master_fix", "/sensing/gnss/master/fix");
    const auto t_rfix = declare_parameter<std::string>("topics.gnss_rover_fix", "/sensing/gnss/rover/fix");
    const auto t_vel = declare_parameter<std::string>("topics.velocity_out", "/result/velocity");
    const auto t_pos = declare_parameter<std::string>("topics.position_out", "/result/position");
    const auto t_status = declare_parameter<std::string>("topics.status_out", "/result/slip_status");
    frame_map_ = declare_parameter<std::string>("frames.map", "map");
    frame_base_ = declare_parameter<std::string>("frames.base_link", "base_link");
    frame_rel_ = declare_parameter<std::string>("frames.relative", "odom");
    min_period_ = declare_parameter<double>("publish.min_period", 0.02);
    time_offset_ = declare_parameter<double>("publish.time_offset", 0.08);
    const double wd_period = declare_parameter<double>("publish.timer_period", 0.025);
    timer_extrap_ = declare_parameter<double>("publish.timer_max_extrapolation", 0.1);
    wd_max_extrap_ = declare_parameter<double>("publish.watchdog_max_extrapolation", 2.0);
    const double diag_period = declare_parameter<double>("publish.diagnostics_period", 0.2);
    const bool closed = declare_parameter<bool>("map.use_closed_route", true);
    auto map_files = declare_parameter<std::vector<std::string>>("map.files", std::vector<std::string>{});
    auto landmarks_file = declare_parameter<std::string>("map.landmarks_file", "");

    // ---------------- route map
    if (map_files.empty()) {
      const std::string share = ament_index_cpp::get_package_share_directory("tram_odometry");
      if (closed) {
        map_files = {share + "/maps/route_closed.json"};
      } else {
        map_files = {share + "/maps/pathgraph_tallinskaya_shchukinskaya.json",
          share + "/maps/pathgraph_shchukinskaya_tallinskaya.json"};
      }
    }
    for (const auto & f : map_files) {
      try {
        route_.load_file(f);
        RCLCPP_INFO(get_logger(), "loaded route map %s", f.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "cannot load map %s: %s", f.c_str(), e.what());
      }
    }
    if (route_.empty()) {
      RCLCPP_WARN(get_logger(), "no route map: publishing relative odometry in frame '%s'",
        frame_rel_.c_str());
    }
    est_ = std::make_unique<Estimator>(ep, tp, gp, &route_);
    if (landmarks_file.empty() && closed) {
      landmarks_file = ament_index_cpp::get_package_share_directory("tram_odometry") +
        "/maps/stations.json";
    }
    if (!landmarks_file.empty() && ep.landmarks_enabled) {
      try {
        est_->set_landmarks(load_landmarks(landmarks_file));
        RCLCPP_INFO(get_logger(), "loaded landmarks %s", landmarks_file.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "no landmarks (%s): %s", landmarks_file.c_str(), e.what());
      }
    }

    // ---------------- I/O
    const auto qos_in = rclcpp::SensorDataQoS().keep_last(50);
    sub_front_ = create_subscription<VelocitySensor>(t_front, qos_in,
        [this](const VelocitySensor::SharedPtr m) {on_wheel(*m, 0);});
    sub_rear_ = create_subscription<VelocitySensor>(t_rear, qos_in,
        [this](const VelocitySensor::SharedPtr m) {on_wheel(*m, 1);});
#if TRAM_HAVE_DRIVER_CMD
    using Cmd = tram_vehicle_msgs::msg::DriverControllerCommand;
    sub_cmd_ = create_subscription<Cmd>(t_cmd, qos_in, [this](const Cmd::SharedPtr m) {
          const auto t0 = SteadyClock::now();
          est_->on_command(stamp_or_now(m->header.stamp), static_cast<double>(m->position));
          after_input(t0);
        });
#else
    (void)t_cmd;
    RCLCPP_WARN(get_logger(), "tram_vehicle_msgs has no DriverControllerCommand: running without "
      "controller input (model uses resistance + disturbance only)");
#endif
    const auto qos_gnss = rclcpp::SensorDataQoS().keep_last(10);
    sub_mfix_ = create_subscription<sensor_msgs::msg::NavSatFix>(t_mfix, qos_gnss,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr m) {on_fix(*m, true);});
    sub_rfix_ = create_subscription<sensor_msgs::msg::NavSatFix>(t_rfix, qos_gnss,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr m) {on_fix(*m, false);});

    pub_vel_ = create_publisher<VelocitySensor>(t_vel, 10);
    pub_pos_ = create_publisher<nav_msgs::msg::Odometry>(t_pos, 10);
    pub_status_ = create_publisher<std_msgs::msg::String>(t_status, 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    watchdog_ = create_wall_timer(std::chrono::duration<double>(wd_period), [this]() {on_watchdog();});
    diag_timer_ = create_wall_timer(std::chrono::duration<double>(diag_period), [this]() {publish_diag();});
    last_input_wall_ = SteadyClock::now();
    RCLCPP_INFO(get_logger(), "tram_odometry_node started (driver command input: %s)",
      TRAM_HAVE_DRIVER_CMD ? "yes" : "no");
  }

private:
  RouteMap route_;
  std::unique_ptr<Estimator> est_;
  std::string frame_map_, frame_base_, frame_rel_;
  double min_period_{0.02};
  double time_offset_{0.08};
  double wd_max_extrap_{2.0};
  double timer_extrap_{0.1};
  double last_pub_t_{-1e18};
  SteadyClock::time_point last_input_wall_;
  // latency statistics (input callback entry -> publish), reset every diagnostics period
  std::vector<double> lat_us_;
  double lat_max_us_{0.0};
  unsigned long n_pub_{0};

  rclcpp::Subscription<VelocitySensor>::SharedPtr sub_front_, sub_rear_;
#if TRAM_HAVE_DRIVER_CMD
  rclcpp::Subscription<tram_vehicle_msgs::msg::DriverControllerCommand>::SharedPtr sub_cmd_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_mfix_, sub_rfix_;
  rclcpp::Publisher<VelocitySensor>::SharedPtr pub_vel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pos_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::TimerBase::SharedPtr watchdog_, diag_timer_;

  double stamp_or_now(const builtin_interfaces::msg::Time & st)
  {
    if (st.sec == 0 && st.nanosec == 0) {return now().seconds();}
    return static_cast<double>(st.sec) + 1e-9 * static_cast<double>(st.nanosec);
  }

  static builtin_interfaces::msg::Time to_stamp(double t)
  {
    builtin_interfaces::msg::Time s;
    const double sec = std::floor(t);
    s.sec = static_cast<int32_t>(sec);
    s.nanosec = static_cast<uint32_t>(std::min(999999999.0, std::round((t - sec) * 1e9)));
    return s;
  }

  void on_wheel(const VelocitySensor & m, int wheel)
  {
    const auto t0 = SteadyClock::now();
    est_->on_wheel(stamp_or_now(m.header.stamp), wheel, m.velocity);
    after_input(t0);
  }

  void on_fix(const sensor_msgs::msg::NavSatFix & m, bool master)
  {
    est_->on_gnss_fix(stamp_or_now(m.header.stamp), master, m.latitude, m.longitude, m.altitude,
      m.status.status);
  }

  void after_input(SteadyClock::time_point t0)
  {
    last_input_wall_ = SteadyClock::now();
    if (!est_->started() || est_->time() - last_pub_t_ < min_period_) {return;}
    publish(est_->output_delayed(time_offset_));
    const double us = std::chrono::duration<double, std::micro>(SteadyClock::now() - t0).count();
    if (lat_us_.size() < 4096) {lat_us_.push_back(us);}
    lat_max_us_ = std::max(lat_max_us_, us);
  }

  void on_watchdog()
  {
    // Fixed-rate publication between input events (40 Hz by default) and during input
    // dropouts: model extrapolation of a COPY of the filter, so the real filter state and its
    // time base are never altered. Between inputs the extrapolation is capped at
    // timer_max_extrapolation; during a total dropout it continues up to
    // watchdog_max_extrapolation.
    if (!est_->started()) {return;}
    const double idle = std::chrono::duration<double>(SteadyClock::now() - last_input_wall_).count();
    if (idle > wd_max_extrap_) {return;}
    const double ahead = idle < 0.3 ? std::min(idle, timer_extrap_) : idle;
    Estimator copy = *est_;
    const double t = est_->time() + ahead;
    if (t - last_pub_t_ < min_period_ || !copy.advance_to(t)) {return;}
    publish(copy.output_delayed(time_offset_));
  }

  void publish(const EstimatorOutput & o)
  {
    last_pub_t_ = o.t;
    ++n_pub_;
    const auto stamp = to_stamp(o.t);
    VelocitySensor v;
    v.header.stamp = stamp;
    v.header.frame_id = frame_base_;
    v.velocity = o.v;
    pub_vel_->publish(v);
    if (!o.position_valid) {return;}  // position is published once initialised
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = o.relative ? frame_rel_ : frame_map_;
    od.child_frame_id = frame_base_;
    od.pose.pose.position.x = o.x;
    od.pose.pose.position.y = o.y;
    od.pose.pose.position.z = o.z;
    od.pose.pose.orientation.z = std::sin(0.5 * o.yaw);
    od.pose.pose.orientation.w = std::cos(0.5 * o.yaw);
    std::copy(o.pose_cov.begin(), o.pose_cov.end(), od.pose.covariance.begin());
    od.twist.twist.linear.x = o.v;
    od.twist.covariance[0] = o.var_v;
    od.twist.covariance[7] = od.twist.covariance[14] = 1e-4;
    od.twist.covariance[21] = od.twist.covariance[28] = od.twist.covariance[35] = 1e-3;
    pub_pos_->publish(od);
  }

  void publish_diag()
  {
    if (!est_->started()) {return;}
    const EstimatorOutput o = est_->output();
    double p50 = 0.0;
    if (!lat_us_.empty()) {
      std::nth_element(lat_us_.begin(), lat_us_.begin() + static_cast<long>(lat_us_.size() / 2), lat_us_.end());
      p50 = lat_us_[lat_us_.size() / 2];
    }
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
      "{\"t\":%.3f,\"mode\":\"%s\",\"mu\":%.3f,\"front\":{\"mode\":\"%s\",\"p_grip\":%.3f,\"p_slip\":%.3f,"
      "\"p_fault\":%.3f,\"slip_ratio\":%.3f,\"stale\":%s},\"rear\":{\"mode\":\"%s\",\"p_grip\":%.3f,"
      "\"p_slip\":%.3f,\"p_fault\":%.3f,\"slip_ratio\":%.3f,\"stale\":%s},\"v\":%.3f,\"a\":%.3f,"
      "\"disturbance\":%.3f,\"kappa_t\":%.3f,\"kappa_b\":%.3f,\"initialized\":%s,\"on_map\":%s,"
      "\"latency_us_p50\":%.1f,\"latency_us_max\":%.1f}",
      o.t, to_string(o.mode), o.mu,
      wheel_mode_name(argmax(o.p_mode[0])), o.p_mode[0][0], o.p_mode[0][1], o.p_mode[0][2],
      o.slip_ratio[0], o.stale[0] ? "true" : "false",
      wheel_mode_name(argmax(o.p_mode[1])), o.p_mode[1][0], o.p_mode[1][1], o.p_mode[1][2],
      o.slip_ratio[1], o.stale[1] ? "true" : "false", o.v, o.a, o.d, o.kappa_t, o.kappa_b,
      o.initialized ? "true" : "false", o.on_map ? "true" : "false", p50, lat_max_us_);
    std_msgs::msg::String s;
    s.data = buf;
    pub_status_->publish(s);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = to_stamp(o.t);
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "tram_odometry: wheel odometry / slip";
    st.hardware_id = "tram";
    st.level = o.mode == GlobalMode::NORMAL_GRIP ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      (o.mode == GlobalMode::DEAD_RECKONING ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    st.message = to_string(o.mode);
    auto kv = [&st](const std::string & k, double v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = std::to_string(v);
        st.values.push_back(e);
      };
    kv("mu_hat", o.mu);
    kv("front_p_grip", o.p_mode[0][0]); kv("front_p_slip", o.p_mode[0][1]); kv("front_p_fault", o.p_mode[0][2]);
    kv("rear_p_grip", o.p_mode[1][0]); kv("rear_p_slip", o.p_mode[1][1]); kv("rear_p_fault", o.p_mode[1][2]);
    kv("front_slip_ratio", o.slip_ratio[0]); kv("rear_slip_ratio", o.slip_ratio[1]);
    kv("front_stale", o.stale[0]); kv("rear_stale", o.stale[1]);
    kv("velocity", o.v); kv("velocity_var", o.var_v); kv("accel", o.a);
    kv("along_track_var", o.var_s); kv("disturbance_accel", o.d);
    kv("scale_bias_front", o.bf); kv("scale_bias_rear", o.br); kv("wheel_scale_common", o.c);
    kv("landmark_updates", static_cast<double>(o.landmark_updates));
    kv("last_landmark_innovation_m", o.last_landmark_innovation);
    kv("kappa_traction", o.kappa_t); kv("kappa_brake", o.kappa_b);
    kv("notch", o.notch); kv("notch_filtered", o.notch_f);
    kv("traction_force_N", o.force_N); kv("motor_torque_Nm", o.torque_Nm);
    kv("dropped_messages", static_cast<double>(o.dropped));
    kv("published", static_cast<double>(n_pub_));
    kv("latency_us_p50", p50); kv("latency_us_max", lat_max_us_);
    arr.status.push_back(st);
    pub_diag_->publish(arr);
    lat_us_.clear();
    lat_max_us_ = 0.0;
  }

  static int argmax(const std::array<double, 3> & p)
  {
    return static_cast<int>(std::max_element(p.begin(), p.end()) - p.begin());
  }
};

}  // namespace tram_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tram_odometry::TramOdometryNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

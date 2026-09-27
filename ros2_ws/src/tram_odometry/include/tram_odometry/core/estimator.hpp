// Longitudinal state estimator: EKF + per-bogie IMM-style mode probabilities.
// ROS-independent; driven by timestamped events (see docs/model.md, section 1.4).
//
// State x = [v, s, d, b_f, b_r, mu, c]
//   v   longitudinal speed of base_link, m/s (>= 0, trams do not reverse in service)
//   s   arc coordinate in the filter domain, m (route position = s + route_offset)
//   d   slowly varying disturbance acceleration (grade + model error), m/s^2
//   b_f, b_r  relative scale bias of front/rear speed sensor (wheel wear), -
//   mu  adhesion coefficient estimate, -
//   c   common odometry scale error (wheel wear / diameter), observable through landmark
//       (repeatable stop location) updates of s
#pragma once

#include <array>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "tram_odometry/core/geo.hpp"
#include "tram_odometry/core/math_util.hpp"
#include "tram_odometry/core/route_map.hpp"
#include "tram_odometry/core/traction_model.hpp"

namespace tram_odometry
{

constexpr int kN = 7;
enum StateIdx { V = 0, S = 1, D = 2, BF = 3, BR = 4, MU = 5, CS = 6 };
enum WheelMode { GRIP = 0, SLIP = 1, FAULT = 2 };
enum class GlobalMode { NORMAL_GRIP = 0, SLIP_DETECTED = 1, SENSOR_FAULT = 2, DEAD_RECKONING = 3 };

const char * to_string(GlobalMode m);
const char * wheel_mode_name(int m);

struct EstimatorParams
{
  // --- inputs
  double wheel_unit_scale{1.0 / 3.6};   // recorded bogie speed is km/h (verified vs reference)
  double wheel_scale_front{1.0};
  double wheel_scale_rear{1.0};
  double max_wheel_speed{30.0};          // m/s, larger values are rejected as outliers
  double arc_scale{1.00043};             // map arc length / odometry distance (calibrated)
  // --- measurement noise
  double sigma_wheel_abs{0.03};
  double sigma_wheel_rel{0.005};
  // --- process noise (continuous, per second)
  double q_v{0.04};
  double q_v_no_cmd{0.30};
  double q_s{1e-4};
  double q_d{0.01};
  double tau_d{30.0};
  double q_b{1e-9};
  double q_mu{1e-5};
  double mu_nominal{0.20};
  double tau_mu{300.0};
  double k_adhesion{1.0};                // share of weight on powered/braked axles
  bool use_adhesion_limit{true};
  // --- initial covariance
  double p0_v{1.0}, p0_s{0.01}, p0_d{0.05}, p0_b{1e-5}, p0_mu{0.01};
  // --- IMM (per bogie) : Markov transition per measurement and mode likelihoods
  double p_grip_stay{0.98}, p_grip_to_slip{0.015};   // remaining -> fault
  double p_slip_stay{0.88}, p_slip_to_grip{0.10};
  double p_fault_stay{0.97}, p_fault_to_grip{0.02};
  double sigma_slip{1.5};                // extra innovation std under slip hypothesis
  double fault_density{1.0 / 30.0};      // uniform likelihood under fault hypothesis
  double p_fault_skip{0.5};              // skip update if P(fault) above
  double p_slip_skip{0.9};               // skip update if P(slip) above
  double nis_hard_gate{100.0};           // chi2(1) hard gate when grip is not dominant
  // --- slip / anomaly heuristics (evidence multipliers)
  double wheel_accel_max{2.5};           // m/s^2 physically plausible |dv/dt|
  double front_rear_abs{0.3};            // m/s
  double front_rear_rel{0.05};
  double directional_thr{0.3};           // m/s: spin in traction / skid in braking
  double stuck_speed{0.5};               // m/s: sensor reads 0 while moving faster
  int var_window{20};
  double var_thr{0.05};                  // (m/s)^2 innovation variance threshold
  double stale_timeout{0.6};             // s without messages -> channel stale
  // --- time handling
  double max_msg_lag{0.5};               // older messages than filter time are dropped
  double reset_on_backstep{2.0};         // time jump backwards -> full reset
  double max_predict_step{0.05};
  // --- initialisation / GNSS
  std::string init_mode{"gnss"};         // gnss | fixed_xy | fixed_s | relative
  double init_x{std::numeric_limits<double>::quiet_NaN()};
  double init_y{std::numeric_limits<double>::quiet_NaN()};
  double init_yaw{std::numeric_limits<double>::quiet_NaN()};
  int init_path{0};
  double init_s{0.0};
  double init_window{5.0};               // s of GNSS averaging after the first fix
  int init_min_fixes{3};
  double init_timeout{15.0};             // s: no GNSS -> fallback (fixed_xy if given, else relative)
  double init_sigma_s{5.0};              // m, along-track std after GNSS alignment (map/platform ambiguity)
  double init_max_lateral{200.0};        // m (off-map starts snap to nearest route point)
  double master_x{-9.873}, master_y{0.0}, master_z{3.0};
  double rover_x{2.563}, rover_y{0.0}, rover_z{3.0};
  bool gnss_correction{false};           // use later GNSS fixes as measurements (off: init only)
  double sigma_gnss_along{1.5};
  double gnss_gate{16.0};
  double lateral_sigma{0.3};             // cross-track std of the route map, m
  // --- common odometry scale + landmark (repeatable stop location) updates
  double p0_c{2.5e-5};                   // (0.5 %)^2 prior on wheel-scale error
  double q_c{1e-10};
  bool landmarks_enabled{true};
  double landmark_standstill_time{3.0};  // s of confirmed standstill before snapping
  double landmark_min_travel{30.0};      // m travelled since the previous snap
  double landmark_gate{8.0};             // m, max |innovation|
  double landmark_stop_sigma{1.0};       // m, stop-position variability added to cluster sigma
};

struct Landmark
{
  double s{0.0};
  double sigma{0.5};
};

/// Reads maps/stations.json ({stations: [{s, sigma, ...}]}). Throws on malformed input.
std::vector<Landmark> load_landmarks(const std::string & file);

struct EstimatorOutput
{
  double t{0.0};
  double v{0.0}, a{0.0};
  double s_filter{0.0};
  double s_route{0.0};
  int path{-1};
  bool position_valid{false};
  bool relative{false};
  bool on_map{false};
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
  double var_v{0.0}, var_s{0.0};
  std::array<double, 36> pose_cov{};
  GlobalMode mode{GlobalMode::NORMAL_GRIP};
  std::array<std::array<double, 3>, 2> p_mode{};  // [wheel][GRIP,SLIP,FAULT]
  std::array<bool, 2> stale{{false, false}};
  std::array<double, 2> slip_ratio{{0.0, 0.0}};   // lambda = (v_wheel - v)/max(v, eps)
  std::array<double, 2> innovation{{0.0, 0.0}};
  double mu{0.0}, d{0.0}, bf{0.0}, br{0.0}, c{0.0};
  unsigned long landmark_updates{0};
  double last_landmark_innovation{0.0};
  double kappa_t{1.0}, kappa_b{1.0};
  double notch{0.0}, notch_f{0.0};
  double force_N{0.0}, torque_Nm{0.0};
  bool has_command{false};
  bool initialized{false};
  unsigned long dropped{0};
};

class Estimator
{
public:
  Estimator(const EstimatorParams & ep, const TractionParams & tp, const GeoParams & gp,
    const RouteMap * route);

  void on_command(double t, double notch);
  /// wheel: 0 = front bogie, 1 = rear bogie; raw value in sensor units (km/h).
  void on_wheel(double t, int wheel, double raw);
  void on_gnss_fix(double t, bool master, double lat, double lon, double alt, int status);
  /// Landmarks (arc length on closed path 0) - see tools/build_stations.py.
  void set_landmarks(std::vector<Landmark> lm) {landmarks_ = std::move(lm);}
  /// Predict to time t without a measurement (timer path). Returns false if t is stale.
  bool advance_to(double t);

  bool started() const {return started_;}
  double time() const {return t_;}
  EstimatorOutput output() const;
  /// Output stamped at the current filter time but describing the state delay seconds
  /// earlier (time-offset calibration against the reference timeline, see docs/validation.md).
  EstimatorOutput output_delayed(double delay) const;
  void reset();

private:
  using Vec = Mat<kN, 1>;
  using Cov = Mat<kN, kN>;

  EstimatorParams p_;
  GeoParams geo_;
  const RouteMap * route_;
  TractionModel model_;

  bool started_{false};
  double t_{0.0};
  Vec x_;
  Cov P_;
  double a_last_{0.0};
  double a_filt_{0.0};
  double v_prev_post_{0.0};
  double t_prev_post_{-1.0};
  unsigned long dropped_{0};
  double t_start_{0.0};

  struct WheelState
  {
    std::array<double, 3> p{{1.0, 0.0, 0.0}};
    double last_t{-1.0};
    double last_z{std::numeric_limits<double>::quiet_NaN()};
    double innov{0.0};
    std::deque<double> innov_hist;
  };
  std::array<WheelState, 2> w_;
  std::deque<std::array<double, 3>> hist_;   // (t, v, s_filter), last ~2 s
  void record();

  // --- initialisation state
  bool init_done_{false};
  bool init_have_offset_{false};
  bool relative_{false};
  int path_{-1};
  double offset_{0.0};
  double t_first_fix_{-1.0};
  std::deque<double> offsets_;
  std::deque<std::pair<double, double>> s_hist_;   // (t, s_filter)
  struct Fix { double t, x, y, z; };
  std::deque<Fix> master_hist_;
  Fix last_rover_{-1e9, 0, 0, 0};

  // --- landmarks
  std::vector<Landmark> landmarks_;
  double standstill_since_{-1.0};
  double s_last_snap_{-1e9};
  bool snapped_this_stop_{false};
  unsigned long landmark_updates_{0};
  double last_lm_innov_{0.0};
  void landmark_update();

  bool handle_time(double t);
  void predict(double dt);
  Vec f(const Vec & x, double dt) const;
  void scalar_update(const Mat<1, kN> & H, double y, double R);
  void wheel_update(int wheel, double z, double t_meas);
  void post_update_bookkeeping();
  double s_at(double t) const;
  void init_fixed();
  void try_fallback_init();
  void process_base_fix(double t, double bx, double by, double heading, bool heading_known);
};

}  // namespace tram_odometry

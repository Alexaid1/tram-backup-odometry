// Single source of truth for configurable parameters: the ROS node declares every entry
// as a ROS parameter, the offline replay tool reads the same YAML file.
#pragma once

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "tram_odometry/core/estimator.hpp"

namespace tram_odometry
{

struct ParamRef
{
  enum class Kind { Double, Int, Bool, String };
  std::string name;
  Kind kind;
  void * ptr;
};

inline std::vector<ParamRef> make_param_table(EstimatorParams & e, TractionParams & t,
  GeoParams & g)
{
  using K = ParamRef::Kind;
  return {
    // traction / dynamics model
    {"model.A_t", K::Double, &t.A_t}, {"model.gamma_t", K::Double, &t.gamma_t},
    {"model.v_base", K::Double, &t.v_base}, {"model.p_shape", K::Double, &t.p_shape},
    {"model.A_b", K::Double, &t.A_b}, {"model.gamma_b", K::Double, &t.gamma_b},
    {"model.v_eps_brake", K::Double, &t.v_eps_brake},
    {"model.c0", K::Double, &t.c0}, {"model.c1", K::Double, &t.c1}, {"model.c2", K::Double, &t.c2},
    {"model.tau_drive", K::Double, &t.tau_drive}, {"model.notch_max", K::Double, &t.notch_max},
    {"model.deadzone", K::Double, &t.deadzone}, {"model.mass_kg", K::Double, &t.mass_kg},
    {"model.wheel_radius_m", K::Double, &t.wheel_radius_m},
    {"model.gear_ratio", K::Double, &t.gear_ratio}, {"model.n_motors", K::Int, &t.n_motors},
    {"model.adapt", K::Bool, &t.adapt}, {"model.rls_lambda", K::Double, &t.rls_lambda},
    {"model.kappa_min", K::Double, &t.kappa_min}, {"model.kappa_max", K::Double, &t.kappa_max},
    {"model.rls_min_regressor", K::Double, &t.rls_min_regressor},
    // inputs
    {"input.wheel_unit_scale", K::Double, &e.wheel_unit_scale},
    {"input.wheel_scale_front", K::Double, &e.wheel_scale_front},
    {"input.wheel_scale_rear", K::Double, &e.wheel_scale_rear},
    {"input.max_wheel_speed", K::Double, &e.max_wheel_speed},
    {"input.arc_scale", K::Double, &e.arc_scale},
    {"input.max_msg_lag", K::Double, &e.max_msg_lag},
    {"input.reset_on_backstep", K::Double, &e.reset_on_backstep},
    {"input.stale_timeout", K::Double, &e.stale_timeout},
    // filter
    {"ekf.sigma_wheel_abs", K::Double, &e.sigma_wheel_abs},
    {"ekf.sigma_wheel_rel", K::Double, &e.sigma_wheel_rel},
    {"ekf.q_v", K::Double, &e.q_v}, {"ekf.q_v_no_cmd", K::Double, &e.q_v_no_cmd},
    {"ekf.q_s", K::Double, &e.q_s}, {"ekf.q_d", K::Double, &e.q_d}, {"ekf.tau_d", K::Double, &e.tau_d},
    {"ekf.q_b", K::Double, &e.q_b}, {"ekf.q_mu", K::Double, &e.q_mu},
    {"ekf.mu_nominal", K::Double, &e.mu_nominal}, {"ekf.tau_mu", K::Double, &e.tau_mu},
    {"ekf.k_adhesion", K::Double, &e.k_adhesion},
    {"ekf.use_adhesion_limit", K::Bool, &e.use_adhesion_limit},
    {"ekf.p0_v", K::Double, &e.p0_v}, {"ekf.p0_s", K::Double, &e.p0_s},
    {"ekf.p0_d", K::Double, &e.p0_d}, {"ekf.p0_b", K::Double, &e.p0_b},
    {"ekf.p0_mu", K::Double, &e.p0_mu},
    {"ekf.max_predict_step", K::Double, &e.max_predict_step},
    // mode logic (IMM + heuristics)
    {"slip.p_grip_stay", K::Double, &e.p_grip_stay}, {"slip.p_grip_to_slip", K::Double, &e.p_grip_to_slip},
    {"slip.p_slip_stay", K::Double, &e.p_slip_stay}, {"slip.p_slip_to_grip", K::Double, &e.p_slip_to_grip},
    {"slip.p_fault_stay", K::Double, &e.p_fault_stay},
    {"slip.p_fault_to_grip", K::Double, &e.p_fault_to_grip},
    {"slip.sigma_slip", K::Double, &e.sigma_slip}, {"slip.fault_density", K::Double, &e.fault_density},
    {"slip.p_fault_skip", K::Double, &e.p_fault_skip}, {"slip.p_slip_skip", K::Double, &e.p_slip_skip},
    {"slip.nis_hard_gate", K::Double, &e.nis_hard_gate},
    {"slip.wheel_accel_max", K::Double, &e.wheel_accel_max},
    {"slip.front_rear_abs", K::Double, &e.front_rear_abs},
    {"slip.front_rear_rel", K::Double, &e.front_rear_rel},
    {"slip.directional_thr", K::Double, &e.directional_thr},
    {"slip.stuck_speed", K::Double, &e.stuck_speed}, {"slip.var_window", K::Int, &e.var_window},
    {"slip.var_thr", K::Double, &e.var_thr},
    // initialisation / GNSS / map
    {"init.mode", K::String, &e.init_mode}, {"init.x", K::Double, &e.init_x},
    {"init.y", K::Double, &e.init_y}, {"init.yaw", K::Double, &e.init_yaw},
    {"init.path", K::Int, &e.init_path}, {"init.s", K::Double, &e.init_s},
    {"init.window", K::Double, &e.init_window}, {"init.min_fixes", K::Int, &e.init_min_fixes},
    {"init.timeout", K::Double, &e.init_timeout}, {"init.max_lateral", K::Double, &e.init_max_lateral},
    {"init.sigma_s", K::Double, &e.init_sigma_s},
    {"gnss.master_x", K::Double, &e.master_x}, {"gnss.master_y", K::Double, &e.master_y},
    {"gnss.master_z", K::Double, &e.master_z}, {"gnss.rover_x", K::Double, &e.rover_x},
    {"gnss.rover_y", K::Double, &e.rover_y}, {"gnss.rover_z", K::Double, &e.rover_z},
    {"gnss.correction", K::Bool, &e.gnss_correction},
    {"gnss.sigma_along", K::Double, &e.sigma_gnss_along}, {"gnss.gate", K::Double, &e.gnss_gate},
    {"gnss.utm_zone", K::Int, &g.utm_zone}, {"gnss.north", K::Bool, &g.north},
    {"gnss.map_offset_x", K::Double, &g.offset_x}, {"gnss.map_offset_y", K::Double, &g.offset_y},
    {"gnss.map_offset_z", K::Double, &g.offset_z},
    {"map.lateral_sigma", K::Double, &e.lateral_sigma},
    {"ekf.p0_c", K::Double, &e.p0_c}, {"ekf.q_c", K::Double, &e.q_c},
    {"landmarks.enabled", K::Bool, &e.landmarks_enabled},
    {"landmarks.standstill_time", K::Double, &e.landmark_standstill_time},
    {"landmarks.min_travel", K::Double, &e.landmark_min_travel},
    {"landmarks.gate", K::Double, &e.landmark_gate},
    {"landmarks.stop_sigma", K::Double, &e.landmark_stop_sigma},
  };
}

/// Apply "name -> text value" pairs (e.g. parsed from YAML) to the table.
inline void apply_params(const std::vector<ParamRef> & table,
  const std::map<std::string, std::string> & values)
{
  for (const auto & p : table) {
    auto it = values.find(p.name);
    if (it == values.end()) {continue;}
    const std::string & v = it->second;
    switch (p.kind) {
      case ParamRef::Kind::Double:
        *static_cast<double *>(p.ptr) = (v == ".nan" || v == "nan") ?
          std::numeric_limits<double>::quiet_NaN() : std::stod(v);
        break;
      case ParamRef::Kind::Int: *static_cast<int *>(p.ptr) = std::stoi(v); break;
      case ParamRef::Kind::Bool: *static_cast<bool *>(p.ptr) = (v == "true" || v == "True" || v == "1");
        break;
      case ParamRef::Kind::String: *static_cast<std::string *>(p.ptr) = v; break;
    }
  }
}

}  // namespace tram_odometry

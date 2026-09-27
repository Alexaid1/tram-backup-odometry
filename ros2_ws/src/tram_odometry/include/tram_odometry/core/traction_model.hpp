// Nonlinear traction/brake drive model + running resistance (docs/model.md, 1.1-1.2).
//
// All forces are expressed as specific forces (per unit of nominal mass, m/s^2); the
// physical force/torque are recovered with mass_kg / wheel radius / gear ratio for
// diagnostics only.
//
//   u_f' = (u - u_f) / tau_drive                       driver notch -> drive response lag
//   a_tr = A_t (u_f/15)^g_t / (1 + (v/v_b)^p)^(1/p)    u_f > deadzone  (const force -> const power)
//   a_br = -A_b (|u_f|/15)^g_b tanh(v/v_eps)           u_f < -deadzone (no reverse push at standstill)
//   a_drive = kappa_t a_tr + kappa_b a_br             kappa_* adapted online by RLS
//   r(v) = (c0 + c1 v + c2 v^2) tanh(v/0.1)            running resistance (rolling + aero)
#pragma once

namespace tram_odometry
{

struct TractionParams
{
  // calibrated offline on the dataset (tools/calibrate.py)
  double A_t{1.5013};
  double gamma_t{0.8239};
  double v_base{5.447};
  double p_shape{2.887};
  double A_b{1.2072};
  double gamma_b{0.5839};
  double v_eps_brake{0.3};
  double c0{0.0534};
  double c1{-0.00584};
  double c2{0.000284};
  double tau_drive{0.3};
  double notch_max{15.0};
  double deadzone{0.5};
  // physical constants (diagnostics / documentation)
  double mass_kg{50000.0};
  double wheel_radius_m{0.355};
  double gear_ratio{7.0};
  int n_motors{4};
  // online adaptation (RLS with forgetting)
  bool adapt{true};
  double rls_lambda{0.9995};
  double kappa_min{0.6};
  double kappa_max{1.6};
  double rls_min_regressor{0.15};
};

class TractionModel
{
public:
  explicit TractionModel(const TractionParams & p = TractionParams{})
  : p_(p) {}

  void set_command(double notch) {u_ = notch; have_cmd_ = true;}
  bool has_command() const {return have_cmd_;}
  void reset();

  /// Advance the drive lag by dt seconds.
  void propagate(double dt);

  /// Unscaled model acceleration components at speed v (m/s).
  double traction_nominal(double v) const;
  double brake_nominal(double v) const;
  /// Adapted drive acceleration kappa_t*a_tr + kappa_b*a_br.
  double drive_accel(double v) const;
  double resistance(double v) const;

  /// RLS update: a_obs is the observed acceleration minus the estimated disturbance.
  void adapt(double a_obs, double v);

  double notch() const {return u_;}
  double notch_filtered() const {return u_f_;}
  double kappa_traction() const {return kappa_t_;}
  double kappa_brake() const {return kappa_b_;}
  /// Equivalent longitudinal force [N] and motor torque [N*m] (per motor) for diagnostics.
  double force_N(double v) const {return p_.mass_kg * drive_accel(v);}
  double motor_torque_Nm(double v) const
  {
    return force_N(v) * p_.wheel_radius_m / (p_.gear_ratio * p_.n_motors);
  }
  const TractionParams & params() const {return p_;}

private:
  TractionParams p_;
  double u_{0.0}, u_f_{0.0};
  bool have_cmd_{false};
  double kappa_t_{1.0}, kappa_b_{1.0};
  double P_t_{0.05}, P_b_{0.05};
};

}  // namespace tram_odometry

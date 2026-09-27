#include "tram_odometry/core/traction_model.hpp"

#include <algorithm>
#include <cmath>

#include "tram_odometry/core/math_util.hpp"

namespace tram_odometry
{

void TractionModel::reset()
{
  u_ = u_f_ = 0.0;
  have_cmd_ = false;
  kappa_t_ = kappa_b_ = 1.0;
  P_t_ = P_b_ = 0.05;
}

void TractionModel::propagate(double dt)
{
  if (dt <= 0.0) {return;}
  const double k = p_.tau_drive > 1e-6 ? 1.0 - std::exp(-dt / p_.tau_drive) : 1.0;
  u_f_ += k * (u_ - u_f_);
}

double TractionModel::traction_nominal(double v) const
{
  if (u_f_ <= p_.deadzone) {return 0.0;}
  const double un = std::min(u_f_, p_.notch_max) / p_.notch_max;
  const double vv = std::max(v, 0.0) / p_.v_base;
  const double g = 1.0 / std::pow(1.0 + std::pow(vv, p_.p_shape), 1.0 / p_.p_shape);
  return p_.A_t * std::pow(un, p_.gamma_t) * g;
}

double TractionModel::brake_nominal(double v) const
{
  if (u_f_ >= -p_.deadzone) {return 0.0;}
  const double un = std::min(-u_f_, p_.notch_max) / p_.notch_max;
  return -p_.A_b * std::pow(un, p_.gamma_b) * std::tanh(std::max(v, 0.0) / p_.v_eps_brake);
}

double TractionModel::drive_accel(double v) const
{
  return kappa_t_ * traction_nominal(v) + kappa_b_ * brake_nominal(v);
}

double TractionModel::resistance(double v) const
{
  const double vp = std::max(v, 0.0);
  return (p_.c0 + p_.c1 * vp + p_.c2 * vp * vp) * std::tanh(vp / 0.1);
}

void TractionModel::adapt(double a_obs, double v)
{
  if (!p_.adapt || !have_cmd_) {return;}
  const double y = a_obs + resistance(v);
  const double phi_t = traction_nominal(v), phi_b = brake_nominal(v);
  // scalar RLS with forgetting factor, one estimator per regime
  auto rls = [&](double phi, double & kappa, double & P) {
      if (std::fabs(phi) < p_.rls_min_regressor) {return;}
      const double K = P * phi / (p_.rls_lambda + phi * P * phi);
      kappa = clampd(kappa + K * (y - kappa * phi), p_.kappa_min, p_.kappa_max);
      P = clampd((P - K * phi * P) / p_.rls_lambda, 1e-5, 1.0);
    };
  if (phi_t > 0.0) {rls(phi_t, kappa_t_, P_t_);}
  if (phi_b < 0.0) {rls(phi_b, kappa_b_, P_b_);}
}

}  // namespace tram_odometry

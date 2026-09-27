#include "tram_odometry/core/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "tram_odometry/core/json_lite.hpp"

namespace tram_odometry
{

std::vector<Landmark> load_landmarks(const std::string & file)
{
  const json::Value root = json::parse_file(file);
  std::vector<Landmark> out;
  for (const auto & e : root.at("stations").arr) {
    Landmark l;
    l.s = e.at("s").number();
    l.sigma = e.has("sigma") ? e.at("sigma").number() : 1.0;
    out.push_back(l);
  }
  return out;
}

const char * to_string(GlobalMode m)
{
  switch (m) {
    case GlobalMode::NORMAL_GRIP: return "normal_grip";
    case GlobalMode::SLIP_DETECTED: return "slip_detected";
    case GlobalMode::SENSOR_FAULT: return "sensor_fault";
    case GlobalMode::DEAD_RECKONING: return "dead_reckoning";
  }
  return "unknown";
}

const char * wheel_mode_name(int m)
{
  return m == GRIP ? "grip" : (m == SLIP ? "slip" : "fault");
}

namespace
{
double gauss(double y, double var)
{
  var = std::max(var, 1e-9);
  return std::exp(-0.5 * y * y / var) / std::sqrt(2.0 * kPi * var);
}

double median(std::vector<double> v)
{
  if (v.empty()) {return 0.0;}
  std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
  return v[v.size() / 2];
}
}  // namespace

Estimator::Estimator(const EstimatorParams & ep, const TractionParams & tp, const GeoParams & gp,
  const RouteMap * route)
: p_(ep), geo_(gp), route_(route), model_(tp)
{
  reset();
}

void Estimator::reset()
{
  started_ = false;
  t_ = 0.0;
  x_ = Vec::zero();
  x_(MU, 0) = p_.mu_nominal;
  P_ = Cov::zero();
  P_(V, V) = p_.p0_v; P_(S, S) = p_.p0_s; P_(D, D) = p_.p0_d;
  P_(BF, BF) = p_.p0_b; P_(BR, BR) = p_.p0_b; P_(MU, MU) = p_.p0_mu; P_(CS, CS) = p_.p0_c;
  standstill_since_ = -1.0;
  s_last_snap_ = -1e9;
  snapped_this_stop_ = false;
  landmark_updates_ = 0;
  last_lm_innov_ = 0.0;
  a_last_ = a_filt_ = 0.0;
  v_prev_post_ = 0.0;
  t_prev_post_ = -1.0;
  model_.reset();
  for (auto & w : w_) {w = WheelState{};}
  init_done_ = init_have_offset_ = relative_ = false;
  path_ = -1;
  offset_ = 0.0;
  t_first_fix_ = -1.0;
  offsets_.clear();
  s_hist_.clear();
  master_hist_.clear();
  last_rover_ = Fix{-1e9, 0, 0, 0};
}

// ------------------------------------------------------------------ time handling
bool Estimator::handle_time(double t)
{
  if (!std::isfinite(t)) {++dropped_; return false;}
  if (!started_) {
    started_ = true;
    t_ = t;
    t_start_ = t;
    init_fixed();
    return true;
  }
  if (t < t_ - p_.reset_on_backstep) {  // bag restarted / clock jumped back
    const unsigned long d = dropped_;
    reset();
    dropped_ = d;
    started_ = true;
    t_ = t;
    t_start_ = t;
    init_fixed();
    return true;
  }
  if (t < t_ - p_.max_msg_lag) {++dropped_; return false;}
  if (t > t_) {
    predict(t - t_);
    t_ = t;
    record();
  }
  try_fallback_init();
  return true;
}

bool Estimator::advance_to(double t)
{
  if (!started_ || !(t > t_)) {return false;}
  return handle_time(t);
}

// ------------------------------------------------------------------ process model
Estimator::Vec Estimator::f(const Vec & x, double dt) const
{
  Vec y = x;
  const double v = std::max(x(V, 0), 0.0);
  double a_dr = model_.has_command() ? model_.drive_accel(v) : 0.0;
  if (p_.use_adhesion_limit) {
    // smooth adhesion limit |a_drive| <= mu*g*k_adh (wheel-rail friction ellipse, 1D)
    const double lim = std::max(x(MU, 0), 0.02) * kGravity * p_.k_adhesion;
    a_dr = lim * std::tanh(a_dr / lim);
  }
  const double a = a_dr - model_.resistance(v) + x(D, 0);
  const double v1 = std::max(0.0, v + a * dt);
  y(V, 0) = v1;
  // c: common odometry scale error acts on the travelled distance (observable via landmarks)
  y(S, 0) = x(S, 0) + p_.arc_scale * (1.0 + x(CS, 0)) * 0.5 * (v + v1) * dt;
  // at standstill the disturbance is unobservable (static friction): relax it quickly
  y(D, 0) = x(D, 0) * std::exp(-dt / (v < 0.05 ? 1.0 : p_.tau_d));
  y(MU, 0) = p_.mu_nominal + (x(MU, 0) - p_.mu_nominal) * std::exp(-dt / p_.tau_mu);
  return y;
}

void Estimator::predict(double dt)
{
  while (dt > 1e-9) {
    const double h = std::min(dt, p_.max_predict_step);
    dt -= h;
    model_.propagate(h);
    const Vec x0 = x_;
    const Vec x1 = f(x0, h);
    // numerical Jacobian (central differences)
    Cov F = Cov::zero();
    const double eps[kN] = {1e-4, 1e-3, 1e-4, 1e-6, 1e-6, 1e-4, 1e-6};
    for (int j = 0; j < kN; ++j) {
      Vec xp = x0, xm = x0;
      xp(j, 0) += eps[j];
      xm(j, 0) -= eps[j];
      const Vec fp = f(xp, h), fm = f(xm, h);
      for (int i = 0; i < kN; ++i) {F(i, j) = (fp(i, 0) - fm(i, 0)) / (2.0 * eps[j]);}
    }
    Cov Q = Cov::zero();
    Q(V, V) = (model_.has_command() ? p_.q_v : p_.q_v_no_cmd) * h;
    Q(S, S) = p_.q_s * h;
    Q(D, D) = p_.q_d * h;
    Q(BF, BF) = p_.q_b * h;
    Q(BR, BR) = p_.q_b * h;
    Q(MU, MU) = p_.q_mu * h;
    Q(CS, CS) = p_.q_c * h;
    a_last_ = (x1(V, 0) - x0(V, 0)) / h;
    x_ = x1;
    P_ = F * P_ * F.t() + Q;
    t_ += h;
    record();
  }
  s_hist_.emplace_back(t_, x_(S, 0));
  while (s_hist_.size() > 2 && s_hist_.front().first < t_ - 60.0) {s_hist_.pop_front();}
}

void Estimator::record()
{
  if (!hist_.empty() && hist_.back()[0] >= t_) {hist_.back() = {t_, x_(V, 0), x_(S, 0)}; return;}
  hist_.push_back({t_, x_(V, 0), x_(S, 0)});
  while (hist_.size() > 2 && hist_.front()[0] < t_ - 2.0) {hist_.pop_front();}
}

void Estimator::scalar_update(const Mat<1, kN> & H, double y, double R)
{
  const Mat<kN, 1> PHt = P_ * H.t();
  const double Sv = (H * PHt)(0, 0) + R;
  if (!(Sv > 0.0) || !std::isfinite(y)) {return;}
  const Mat<kN, 1> K = PHt * (1.0 / Sv);
  for (int i = 0; i < kN; ++i) {x_(i, 0) += K(i, 0) * y;}
  // Joseph form for numerical robustness
  const Cov I_KH = Cov::identity() - K * H;
  P_ = I_KH * P_ * I_KH.t() + (K * K.t()) * R;
  x_(V, 0) = std::max(0.0, x_(V, 0));
  x_(MU, 0) = clampd(x_(MU, 0), 0.02, 0.6);
  x_(BF, 0) = clampd(x_(BF, 0), -0.05, 0.05);
  x_(BR, 0) = clampd(x_(BR, 0), -0.05, 0.05);
  x_(CS, 0) = clampd(x_(CS, 0), -0.05, 0.05);
}

// ------------------------------------------------------------------ inputs
void Estimator::on_command(double t, double notch)
{
  if (!std::isfinite(notch) || std::fabs(notch) > 100.0) {++dropped_; return;}
  if (!handle_time(t)) {return;}
  model_.set_command(clampd(notch, -model_.params().notch_max, model_.params().notch_max));
}

void Estimator::on_wheel(double t, int wheel, double raw)
{
  if (wheel < 0 || wheel > 1) {return;}
  const double z = raw * p_.wheel_unit_scale * (wheel == 0 ? p_.wheel_scale_front : p_.wheel_scale_rear);
  if (!std::isfinite(z) || z < -0.5 || z > p_.max_wheel_speed) {++dropped_; return;}
  // messages slightly older than filter time (<= max_msg_lag) are applied with lag
  // compensation; older ones are dropped; large backward jumps reset the filter.
  if (!handle_time(t)) {return;}
  wheel_update(wheel, std::max(z, 0.0), std::min(t, t_));
  post_update_bookkeeping();
  record();
}

void Estimator::wheel_update(int k, double z, double t_meas)
{
  WheelState & w = w_[static_cast<size_t>(k)];
  WheelState & o = w_[static_cast<size_t>(1 - k)];
  const int bi = (k == 0) ? BF : BR;
  const double lag = clampd(t_ - t_meas, 0.0, p_.max_msg_lag);
  const double v = x_(V, 0);
  const double b = x_(bi, 0);
  const double v_lag = std::max(0.0, v - a_last_ * lag);
  const double z_pred = (1.0 + b) * v_lag;
  Mat<1, kN> H = Mat<1, kN>::zero();
  H(0, V) = (1.0 + b);
  H(0, bi) = v_lag;
  const double R0 = p_.sigma_wheel_abs * p_.sigma_wheel_abs +
    std::pow(p_.sigma_wheel_rel * v, 2);
  const double HPH = (H * P_ * H.t())(0, 0);
  double Sn = HPH + R0;
  const double y = z - z_pred;
  w.innov = y;

  // ---------------- evidence from heuristics (docs/model.md 1.3)
  double ev_grip = 1.0, ev_slip = 1.0, ev_fault = 1.0;
  const bool o_fresh = o.last_t >= 0.0 && (t_ - o.last_t) < p_.stale_timeout;
  // (1) stuck at zero while the other bogie moves. A rigid vehicle cannot have one bogie at
  // standstill while the other rolls at > stuck_speed; a locked wheel (skid) is excluded
  // unless braking is commanded. Independent of the current estimate (which a stuck channel
  // could drag down).
  if (z < 0.05 && o_fresh && o.last_z > p_.stuck_speed && o.p[FAULT] < 0.5 &&
    (model_.notch_filtered() > -1.0 || v > p_.stuck_speed))
  {
    ev_fault *= 50.0; ev_grip *= 0.01;
  }
  // (2) physically implausible wheel acceleration (sudden spin / lock)
  bool plausible = true;
  double a_wheel = std::numeric_limits<double>::quiet_NaN();
  if (w.last_t >= 0.0 && std::isfinite(w.last_z)) {
    const double dtw = t_meas - w.last_t;
    if (dtw > 1e-3 && dtw < 1.0) {
      a_wheel = (z - w.last_z) / dtw;
      if (std::fabs(a_wheel) > p_.wheel_accel_max) {
        ev_slip *= 10.0;
        plausible = false;
      }
    }
  }
  // (0) consensus: both bogies fresh, mutually consistent and physically plausible ->
  // the model (not the sensors) is wrong: trust sensors, inflate P_vv (fading memory).
  // A sole remaining channel (the other one faulty/stale) with plausible dynamics is
  // treated the same way: the filter keeps running on the remaining sensor.
  const bool other_bad = !o_fresh || o.p[FAULT] > 0.5;
  const bool consensus = plausible && ((o_fresh && std::isfinite(o.last_z) &&
    o.p[FAULT] < 0.5 && std::fabs(z - o.last_z) < p_.front_rear_abs + p_.front_rear_rel * v) ||
    (other_bad && std::isfinite(a_wheel)));
  if (consensus) {
    ev_grip *= 20.0;
    ev_slip *= 0.2;
    ev_fault *= 0.2;
    if (y * y > 9.0 * Sn) {
      P_(V, V) = std::max(P_(V, V), y * y);
      Sn = (H * P_ * H.t())(0, 0) + R0;
    }
  }
  // (3) front/rear disagreement: blame the channel farther from the prediction
  if (o_fresh && std::isfinite(o.last_z) && o.last_z > 0.05 && o.p[FAULT] < 0.5) {
    const double dfr = std::fabs(z - o.last_z);
    if (dfr > p_.front_rear_abs + p_.front_rear_rel * v && std::fabs(y) > std::fabs(o.innov)) {
      ev_slip *= 5.0;
    }
  }
  // (4) directional: spin under traction (wheel faster and accelerating faster than the
  // model allows), skid / juz under braking (wheel slower and decelerating faster)
  const double uf = model_.notch_filtered();
  const double a_model = model_.drive_accel(v) - model_.resistance(v) + x_(D, 0);
  if (std::isfinite(a_wheel)) {
    if (uf > 1.0 && y > p_.directional_thr && a_wheel > a_model + 0.5) {ev_slip *= 3.0;}
    if (uf < -1.0 && y < -p_.directional_thr && a_wheel < a_model - 0.5) {ev_slip *= 3.0;}
  }
  // (5) abnormal innovation variance
  w.innov_hist.push_back(y);
  while (static_cast<int>(w.innov_hist.size()) > p_.var_window) {w.innov_hist.pop_front();}
  if (static_cast<int>(w.innov_hist.size()) == p_.var_window) {
    double m = 0.0, m2 = 0.0;
    for (double e : w.innov_hist) {m += e; m2 += e * e;}
    m /= p_.var_window;
    const double var = m2 / p_.var_window - m * m;
    if (var > p_.var_thr) {ev_slip *= 2.0;}
  }

  // ---------------- IMM-style mode probability update (Markov prior x likelihood)
  const double Pi[3][3] = {
    {p_.p_grip_stay, p_.p_grip_to_slip, 1.0 - p_.p_grip_stay - p_.p_grip_to_slip},
    {p_.p_slip_to_grip, p_.p_slip_stay, 1.0 - p_.p_slip_stay - p_.p_slip_to_grip},
    {p_.p_fault_to_grip, 1.0 - p_.p_fault_stay - p_.p_fault_to_grip, p_.p_fault_stay}};
  std::array<double, 3> prior{};
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {prior[static_cast<size_t>(j)] += Pi[i][j] * w.p[static_cast<size_t>(i)];}
  }
  const double L[3] = {
    gauss(y, Sn) * ev_grip,
    gauss(y, Sn + p_.sigma_slip * p_.sigma_slip) * ev_slip,
    p_.fault_density * ev_fault};
  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    w.p[static_cast<size_t>(i)] = prior[static_cast<size_t>(i)] * L[i];
    sum += w.p[static_cast<size_t>(i)];
  }
  for (auto & pi : w.p) {pi = std::max(pi / std::max(sum, 1e-300), 1e-4);}
  sum = w.p[0] + w.p[1] + w.p[2];
  for (auto & pi : w.p) {pi /= sum;}

  w.last_t = t_meas;
  w.last_z = z;

  // ---------------- robust (probability-weighted) EKF update
  const double nis = y * y / Sn;
  const bool skip = w.p[FAULT] > p_.p_fault_skip || w.p[SLIP] > p_.p_slip_skip ||
    (nis > p_.nis_hard_gate && w.p[GRIP] < 0.5);
  if (!skip) {
    const double R_eff = R0 / std::max(w.p[GRIP], 1e-3);
    scalar_update(H, y, R_eff);
  }
}

void Estimator::post_update_bookkeeping()
{
  // common-mode of the two scale biases is unobservable without an absolute reference:
  // anchor it with a weak pseudo-measurement (b_f + b_r)/2 = 0.
  Mat<1, kN> H = Mat<1, kN>::zero();
  H(0, BF) = 0.5; H(0, BR) = 0.5;
  scalar_update(H, -(0.5 * x_(BF, 0) + 0.5 * x_(BR, 0)), 1e-8);

  // filtered acceleration of the estimate (used by RLS and adhesion estimation)
  if (t_prev_post_ >= 0.0 && t_ - t_prev_post_ > 0.02) {
    const double dt = t_ - t_prev_post_;
    const double a = (x_(V, 0) - v_prev_post_) / dt;
    const double k = 1.0 - std::exp(-dt / 0.5);
    a_filt_ += k * (a - a_filt_);
    v_prev_post_ = x_(V, 0);
    t_prev_post_ = t_;
  } else if (t_prev_post_ < 0.0) {
    v_prev_post_ = x_(V, 0);
    t_prev_post_ = t_;
  }

  const bool grip0 = w_[0].p[GRIP] > 0.8, grip1 = w_[1].p[GRIP] > 0.8;
  const double v = x_(V, 0);
  // online adaptation of the drive gain (effective mass) in trusted conditions only
  if (grip0 && grip1 && v > 1.0) {model_.adapt(a_filt_ - x_(D, 0), v);}

  // adhesion: (a) demonstrated adhesion is a lower bound on mu,
  //           (b) during slip the transmitted force is adhesion-limited -> mu measurement
  const double demonstrated = std::fabs(a_filt_ + (a_filt_ < 0 ? -1.0 : 1.0) * model_.resistance(v)) /
    (kGravity * p_.k_adhesion);
  Mat<1, kN> Hm = Mat<1, kN>::zero();
  Hm(0, MU) = 1.0;
  const bool slipping = (w_[0].p[SLIP] > 0.5 || w_[1].p[SLIP] > 0.5) &&
    std::fabs(model_.notch_filtered()) > 1.0 && v > 0.5;
  if (slipping) {
    scalar_update(Hm, demonstrated - x_(MU, 0), 0.02 * 0.02);
  } else if (demonstrated > x_(MU, 0) && v > 1.0) {
    scalar_update(Hm, demonstrated - x_(MU, 0), 0.01 * 0.01);
  }
  landmark_update();
}

// ------------------------------------------------------------------ GNSS initialisation
double Estimator::s_at(double t) const
{
  if (s_hist_.empty()) {return x_(S, 0);}
  if (t <= s_hist_.front().first) {return s_hist_.front().second;}
  if (t >= s_hist_.back().first) {return x_(S, 0);}
  for (size_t i = 1; i < s_hist_.size(); ++i) {
    if (s_hist_[i].first >= t) {
      const auto & a = s_hist_[i - 1];
      const auto & b = s_hist_[i];
      const double r = (t - a.first) / std::max(b.first - a.first, 1e-9);
      return a.second + r * (b.second - a.second);
    }
  }
  return x_(S, 0);
}

void Estimator::init_fixed()
{
  if (p_.init_mode == "relative" || !route_ || route_->empty()) {
    relative_ = true; init_done_ = true; init_have_offset_ = true; path_ = -1; offset_ = 0.0;
    return;
  }
  if (p_.init_mode == "fixed_s") {
    path_ = std::clamp(p_.init_path, 0, static_cast<int>(route_->size()) - 1);
    offset_ = p_.init_s;
    init_done_ = init_have_offset_ = true;
  } else if (p_.init_mode == "fixed_xy" && std::isfinite(p_.init_x) && std::isfinite(p_.init_y)) {
    const Projection pr = route_->project(p_.init_x, p_.init_y, p_.init_yaw, 1.2);
    if (pr.path >= 0) {
      path_ = pr.path; offset_ = pr.s; init_done_ = init_have_offset_ = true;
    }
  }
}

void Estimator::try_fallback_init()
{
  if (init_have_offset_ || t_ - t_start_ < p_.init_timeout) {return;}
  // no GNSS within the timeout
  if (std::isfinite(p_.init_x) && std::isfinite(p_.init_y) && route_ && !route_->empty()) {
    const Projection pr = route_->project(p_.init_x, p_.init_y, p_.init_yaw, 1.2);
    if (pr.path >= 0) {
      path_ = pr.path; offset_ = pr.s - x_(S, 0); init_done_ = init_have_offset_ = true;
      return;
    }
  }
  relative_ = true; path_ = -1; offset_ = 0.0; init_done_ = init_have_offset_ = true;
}

void Estimator::on_gnss_fix(double t, bool master, double lat, double lon, double alt, int status)
{
  if (!started_ || status < 0 || !std::isfinite(lat) || !std::isfinite(lon)) {return;}
  if (std::fabs(lat) < 1e-6 && std::fabs(lon) < 1e-6) {return;}
  if (init_done_ && !p_.gnss_correction) {return;}
  if (!route_ || route_->empty() || relative_) {return;}
  if (t > t_ + 1.0 || t < t_ - 5.0) {return;}   // implausible stamp
  const MapPoint m = wgs84_to_map(lat, lon, alt, geo_);
  if (!master) {
    last_rover_ = Fix{t, m.x, m.y, m.z};
    return;
  }
  master_hist_.push_back(Fix{t, m.x, m.y, m.z});
  while (master_hist_.size() > 2 && master_hist_.front().t < t - 5.0) {master_hist_.pop_front();}

  // heading: dual antenna (rover is ahead of master) -> motion -> nearest track tangent
  double heading = std::numeric_limits<double>::quiet_NaN();
  bool known = false;
  const double base = std::hypot(p_.rover_x - p_.master_x, p_.rover_y - p_.master_y);
  if (std::fabs(last_rover_.t - t) < 0.25) {
    const double dx = last_rover_.x - m.x, dy = last_rover_.y - m.y;
    const double L = std::hypot(dx, dy);
    if (std::fabs(L - base) < 2.0) {
      heading = std::atan2(dy, dx) - std::atan2(p_.rover_y - p_.master_y, p_.rover_x - p_.master_x);
      known = true;
    }
  }
  if (!known && x_(V, 0) > 1.0) {
    for (const Fix & q : master_hist_) {
      if (t - q.t > 1.0 && std::hypot(m.x - q.x, m.y - q.y) > 3.0) {
        heading = std::atan2(m.y - q.y, m.x - q.x);
        known = true;
        break;
      }
    }
  }
  if (!known) {
    // GNSS (0.2-1 m) is well below the 3.5 m track spacing: nearest track gives direction
    const Projection pr0 = route_->project(m.x, m.y);
    if (pr0.path < 0) {return;}
    heading = route_->pose(pr0.path, pr0.s).yaw;
  }
  const double c = std::cos(heading), s = std::sin(heading);
  const double bx = m.x - (c * p_.master_x - s * p_.master_y);
  const double by = m.y - (s * p_.master_x + c * p_.master_y);
  process_base_fix(t, bx, by, heading, known);
}

void Estimator::process_base_fix(double t, double bx, double by, double heading, bool)
{
  Projection pr = route_->project(bx, by, heading, 1.0);
  if (pr.path < 0 || std::fabs(pr.lateral) > p_.init_max_lateral) {return;}
  // Open polylines (original pathgraph only): a start off the map is a terminal loop; the
  // tram will join the polyline that STARTS near here, so place it before that start.
  if (!route_->closed(pr.path) &&
    (pr.s < 0.0 || pr.s > route_->length(pr.path) || std::fabs(pr.lateral) > 20.0))
  {
    double best = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < route_->size(); ++k) {
      const RoutePose st = route_->pose(static_cast<int>(k), 0.0);
      const double d = std::hypot(bx - st.x, by - st.y);
      if (d < best) {best = d; pr.path = static_cast<int>(k); pr.s = -d;}
    }
  }

  if (!init_done_) {
    if (path_ >= 0 && pr.path != path_) {  // switched polyline: restart averaging
      offsets_.clear();
    }
    path_ = pr.path;
    if (t_first_fix_ < 0.0) {t_first_fix_ = t;}
    const double off = pr.s - s_at(t);
    if (!offsets_.empty()) {  // unwrap on closed routes
      offsets_.push_back(offsets_.front() + route_->arc_diff(path_, offsets_.front(), off));
    } else {
      offsets_.push_back(off);
    }
    offset_ = median(std::vector<double>(offsets_.begin(), offsets_.end()));
    init_have_offset_ = true;
    if (t - t_first_fix_ >= p_.init_window && static_cast<int>(offsets_.size()) >= p_.init_min_fixes) {
      init_done_ = true;
      P_(S, S) = p_.init_sigma_s * p_.init_sigma_s;
    }
    return;
  }
  // optional correction mode (disabled by default)
  if (pr.path != path_) {return;}
  const double z_filter = pr.s - offset_;
  const double y = route_->arc_diff(path_, x_(S, 0), z_filter);
  const double R = p_.sigma_gnss_along * p_.sigma_gnss_along;
  const double Sv = P_(S, S) + R;
  if (y * y / Sv > p_.gnss_gate) {return;}
  Mat<1, kN> H = Mat<1, kN>::zero();
  H(0, S) = 1.0;
  scalar_update(H, y, R);
}

// ------------------------------------------------------------------ landmarks
// A confirmed standstill (both bogies fresh and at 0, estimate at 0 for >= standstill_time)
// near a repeatable stop location of the offline map gives an absolute along-track
// measurement. Through the s <-> v <-> c correlations the EKF also learns the common wheel
// scale error c (wheel wear), which reduces drift between landmarks.
void Estimator::landmark_update()
{
  if (!p_.landmarks_enabled || landmarks_.empty() || !init_done_ || relative_ || path_ < 0 ||
    !route_ || !route_->closed(path_))
  {
    return;
  }
  const bool fresh0 = w_[0].last_t >= 0.0 && t_ - w_[0].last_t < p_.stale_timeout;
  const bool fresh1 = w_[1].last_t >= 0.0 && t_ - w_[1].last_t < p_.stale_timeout;
  const bool still = fresh0 && fresh1 && w_[0].last_z < 0.05 && w_[1].last_z < 0.05 &&
    x_(V, 0) < 0.05;
  if (!still) {
    standstill_since_ = -1.0;
    snapped_this_stop_ = false;
    return;
  }
  if (standstill_since_ < 0.0) {standstill_since_ = t_;}
  if (snapped_this_stop_ || t_ - standstill_since_ < p_.landmark_standstill_time) {return;}
  snapped_this_stop_ = true;
  if (std::fabs(x_(S, 0) - s_last_snap_) < p_.landmark_min_travel) {return;}

  const double s_now = x_(S, 0) + offset_;
  double best = std::numeric_limits<double>::infinity(), second = best;
  const Landmark * lm = nullptr;
  for (const Landmark & l : landmarks_) {
    const double d = route_->arc_diff(path_, s_now, l.s);
    if (std::fabs(d) < std::fabs(best)) {
      second = best;
      best = d;
      lm = &l;
    } else if (std::fabs(d) < std::fabs(second)) {
      second = d;
    }
  }
  if (!lm || std::fabs(best) > p_.landmark_gate || std::fabs(second) < p_.landmark_gate) {return;}
  const double R = lm->sigma * lm->sigma + p_.landmark_stop_sigma * p_.landmark_stop_sigma;
  if (best * best / (P_(S, S) + R) > 25.0) {return;}   // chi2 outlier (5 sigma)
  Mat<1, kN> H = Mat<1, kN>::zero();
  H(0, S) = 1.0;
  scalar_update(H, best, R);
  s_last_snap_ = x_(S, 0);
  last_lm_innov_ = best;
  ++landmark_updates_;
}

// ------------------------------------------------------------------ output
EstimatorOutput Estimator::output_delayed(double delay) const
{
  EstimatorOutput o = output();
  if (delay <= 0.0 || hist_.size() < 2) {return o;}
  const double tq = t_ - delay;
  if (tq <= hist_.front()[0]) {return o;}
  size_t i = 1;
  while (i + 1 < hist_.size() && hist_[i][0] < tq) {++i;}
  const auto & a = hist_[i - 1];
  const auto & b = hist_[i];
  const double r = clampd((tq - a[0]) / std::max(b[0] - a[0], 1e-9), 0.0, 1.0);
  o.v = a[1] + r * (b[1] - a[1]);
  const double s_f = a[2] + r * (b[2] - a[2]);
  o.s_route += s_f - o.s_filter;
  o.s_filter = s_f;
  if (relative_) {
    o.x = o.s_route;
  } else if (o.position_valid && route_ && path_ >= 0) {
    const RoutePose rp = route_->pose(path_, o.s_route);
    o.x = rp.x; o.y = rp.y; o.z = rp.z; o.yaw = rp.yaw; o.on_map = rp.on_map;
  }
  return o;
}

EstimatorOutput Estimator::output() const
{
  EstimatorOutput o;
  o.t = t_;
  o.v = x_(V, 0);
  o.a = a_filt_;
  o.s_filter = x_(S, 0);
  o.var_v = P_(V, V);
  o.var_s = P_(S, S);
  o.mu = x_(MU, 0);
  o.d = x_(D, 0);
  o.bf = x_(BF, 0);
  o.br = x_(BR, 0);
  o.c = x_(CS, 0);
  o.landmark_updates = landmark_updates_;
  o.last_landmark_innovation = last_lm_innov_;
  o.kappa_t = model_.kappa_traction();
  o.kappa_b = model_.kappa_brake();
  o.notch = model_.notch();
  o.notch_f = model_.notch_filtered();
  o.has_command = model_.has_command();
  o.force_N = model_.force_N(o.v);
  o.torque_Nm = model_.motor_torque_Nm(o.v);
  o.initialized = init_done_;
  o.dropped = dropped_;

  int usable = 0;
  bool any_slip = false, any_fault = false;
  for (int k = 0; k < 2; ++k) {
    const WheelState & w = w_[static_cast<size_t>(k)];
    o.p_mode[static_cast<size_t>(k)] = w.p;
    o.stale[static_cast<size_t>(k)] = w.last_t < 0.0 || (t_ - w.last_t) > p_.stale_timeout;
    o.innovation[static_cast<size_t>(k)] = w.innov;
    o.slip_ratio[static_cast<size_t>(k)] = std::isfinite(w.last_z) ?
      (w.last_z - o.v) / std::max(o.v, 0.5) : 0.0;
    const int mode = static_cast<int>(std::max_element(w.p.begin(), w.p.end()) - w.p.begin());
    if (o.stale[static_cast<size_t>(k)] || mode == FAULT) {any_fault = true;} else if (mode == SLIP) {
      any_slip = true;
    } else {++usable;}
  }
  if (usable == 0) {
    o.mode = GlobalMode::DEAD_RECKONING;
  } else if (any_fault) {
    o.mode = GlobalMode::SENSOR_FAULT;
  } else if (any_slip) {
    o.mode = GlobalMode::SLIP_DETECTED;
  } else {
    o.mode = GlobalMode::NORMAL_GRIP;
  }

  o.relative = relative_;
  o.path = path_;
  if (relative_) {
    o.position_valid = true;
    o.s_route = x_(S, 0);
    o.x = o.s_route;
    o.y = 0.0;
    o.z = 0.0;
    o.yaw = 0.0;
  } else if (init_have_offset_ && route_ && path_ >= 0) {
    o.position_valid = true;
    o.s_route = x_(S, 0) + offset_;
    const RoutePose rp = route_->pose(path_, o.s_route);
    o.x = rp.x; o.y = rp.y; o.z = rp.z; o.yaw = rp.yaw; o.on_map = rp.on_map;
  }
  // pose covariance: along-track variance rotated into map frame + map cross-track
  const double c = std::cos(o.yaw), s = std::sin(o.yaw);
  const double sa = P_(S, S) + (init_done_ ? 0.0 : 4.0), sl = p_.lateral_sigma * p_.lateral_sigma;
  o.pose_cov.fill(0.0);
  o.pose_cov[0] = sa * c * c + sl * s * s;
  o.pose_cov[1] = o.pose_cov[6] = (sa - sl) * c * s;
  o.pose_cov[7] = sa * s * s + sl * c * c;
  o.pose_cov[14] = 0.25;
  o.pose_cov[21] = o.pose_cov[28] = 1e-3;
  o.pose_cov[35] = 1e-3 + std::pow(0.05, 2);
  return o;
}

}  // namespace tram_odometry

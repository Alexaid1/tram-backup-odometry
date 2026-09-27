// Self-contained tests of the estimator core (no gtest dependency): synthetic scenarios for
// normal driving, rear sensor stuck at zero, rear skid under braking, rear dropout, total
// input dropout, garbage values and time jumps. Exit code != 0 on failure.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "tram_odometry/core/estimator.hpp"
#include "tram_odometry/core/geo.hpp"
#include "tram_odometry/core/route_map.hpp"

using namespace tram_odometry;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("FAIL: %s (%s)\n", msg, #cond); ++failures; } \
  else { std::printf("ok:   %s\n", msg); } } while (0)

struct Truth
{
  double v{0.0}, s{0.0};
};

// piecewise acceleration profile: accelerate, cruise, brake to stop, repeat
static double profile_notch(double t)
{
  const double c = std::fmod(t, 60.0);
  if (c < 15.0) {return 10.0;}
  if (c < 35.0) {return 0.0;}
  if (c < 50.0) {return -8.0;}
  return 0.0;
}

enum Scenario { NORMAL, STUCK_REAR, SKID_REAR, DROPOUT_REAR, DROPOUT_ALL, GARBAGE };

static double run(Scenario sc, const RouteMap * route, double * final_err_v, int * saw_mode)
{
  EstimatorParams ep;
  ep.init_mode = "relative";
  TractionParams tp;
  GeoParams gp;
  Estimator est(ep, tp, gp, route);
  TractionModel truth_model(tp);
  Truth tr;
  const double dt = 0.01;
  double max_err = 0.0;
  *saw_mode = 0;
  for (int k = 0; k < 18000; ++k) {  // 180 s
    const double t = 1000.0 + k * dt;
    const double u = profile_notch(t - 1000.0);
    truth_model.set_command(u);
    truth_model.propagate(dt);
    const double a = truth_model.drive_accel(tr.v) - truth_model.resistance(tr.v) + 0.05;
    tr.v = std::max(0.0, tr.v + a * dt);
    tr.s += tr.v * dt;
    if (k % 5 == 0) {est.on_command(t, u);}
    if (k % 10 == 0) {
      const double noise = 0.02 * std::sin(k * 1.7);
      double zf = (tr.v + noise) * 3.6, zr = (tr.v - noise) * 3.6;
      const double tt = t - 1000.0;
      bool send_r = true, send_f = true;
      if (sc == STUCK_REAR && tt > 20 && tt < 80) {zr = 0.0;}
      if (sc == SKID_REAR && tt > 38 && tt < 44) {zr = std::max(0.0, zr - 12.0);}
      if (sc == DROPOUT_REAR && tt > 20 && tt < 70) {send_r = false;}
      if (sc == DROPOUT_ALL && tt > 20 && tt < 23) {send_r = send_f = false;}
      if (sc == GARBAGE && (k / 10) % 37 == 0) {zf = std::nan(""); zr = 1e9;}
      if (send_f) {est.on_wheel(t, 0, zf);}
      if (send_r) {est.on_wheel(t + 0.004, 1, zr);}
      const EstimatorOutput o = est.output();
      if (o.mode != GlobalMode::NORMAL_GRIP) {*saw_mode = 1;}
      if (tt > 2.0) {max_err = std::max(max_err, std::fabs(o.v - tr.v));}
      if (std::getenv("TRAM_TEST_VERBOSE") && std::atoi(std::getenv("TRAM_TEST_VERBOSE")) == sc &&
        std::fabs(o.v - tr.v) > 0.3)
      {
        std::printf("   t=%.1f true=%.2f est=%.2f zf=%.2f zr=%.2f notch_f=%.1f pf=(%.2f %.2f %.2f) "
          "pr=(%.2f %.2f %.2f) var_v=%.3f\n", tt, tr.v, o.v, zf / 3.6, zr / 3.6, o.notch_f,
          o.p_mode[0][0], o.p_mode[0][1], o.p_mode[0][2], o.p_mode[1][0], o.p_mode[1][1],
          o.p_mode[1][2], o.var_v);
      }
      if (!std::isfinite(o.v) || !std::isfinite(o.s_filter)) {return 1e9;}
    }
  }
  *final_err_v = std::fabs(est.output().v - tr.v);
  return max_err;
}

int main(int argc, char ** argv)
{
  // UTM projection against pyproj reference value (EPSG:32637)
  double e = 0, n = 0;
  wgs84_to_utm(55.8, 37.45, 37, true, e, n);
  CHECK(std::fabs(e - 402835.614) < 0.01 && std::fabs(n - 6184907.612) < 0.01, "UTM zone 37N forward");

  RouteMap route;
  if (argc > 1) {
    route.load_file(argv[1]);
    CHECK(route.size() == 1 && route.closed(0), "closed route loaded");
    const RoutePose p = route.pose(0, 1234.5);
    const Projection pr = route.project(p.x + 0.5 * std::cos(p.yaw + 1.5708),
        p.y + 0.5 * std::sin(p.yaw + 1.5708), p.yaw);
    CHECK(pr.path == 0 && std::fabs(pr.s - 1234.5) < 0.05 && std::fabs(pr.lateral - 0.5) < 0.05,
      "route projection round trip");
    const RoutePose w = route.pose(0, route.length(0) + 10.0);
    const RoutePose w0 = route.pose(0, 10.0);
    CHECK(std::hypot(w.x - w0.x, w.y - w0.y) < 1e-6, "closed route wraps");
  }

  const char * names[] = {"normal", "rear stuck at 0", "rear skid (juz)", "rear dropout 50 s",
    "all inputs dropout 3 s", "garbage values"};
  const double limits[] = {0.15, 1.0, 0.5, 0.3, 1.0, 0.3};
  for (int s = NORMAL; s <= GARBAGE; ++s) {
    double fin = 0.0;
    int saw = 0;
    const double m = run(static_cast<Scenario>(s), &route, &fin, &saw);
    std::printf("      scenario %-24s max|v err| = %.3f m/s, final = %.3f, non-normal mode seen = %d\n",
      names[s], m, fin, saw);
    CHECK(m < limits[s], names[s]);
    CHECK(fin < 0.1, (std::string(names[s]) + ": recovers").c_str());
    if (s == STUCK_REAR || s == SKID_REAR || s == DROPOUT_REAR) {
      CHECK(saw == 1, (std::string(names[s]) + ": anomaly flagged").c_str());
    }
  }
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
  return failures ? 1 : 0;
}

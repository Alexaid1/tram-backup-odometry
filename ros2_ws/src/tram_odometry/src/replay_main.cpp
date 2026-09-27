// Offline replay of the estimator core (no ROS needed): used for validation on the
// whole dataset and for tuning. Events are processed in bag-record order with header
// stamps, exactly as the ROS node receives them during `ros2 bag play`.
//
// usage: tram_odometry_replay <params.yaml> <events.csv> <out.csv> <map.json> [map2.json ...]
// events.csv: kind,t_recv,t_stamp,a,b,c,d   kind: 0 cmd(notch) 1 front(km/h) 2 rear(km/h)
//                                                  3 master fix(lat,lon,alt,status) 4 rover fix
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "tram_odometry/core/estimator.hpp"
#include "tram_odometry/core/params_table.hpp"
#include "tram_odometry/core/route_map.hpp"

using namespace tram_odometry;

// Flattens a ROS 2 params YAML (nested maps) into dotted names; ignores the node and
// ros__parameters levels. Sufficient for our config file (scalars only).
static std::map<std::string, std::string> read_yaml_flat(const std::string & path)
{
  std::map<std::string, std::string> out;
  std::ifstream f(path);
  std::string line;
  std::vector<std::pair<int, std::string>> stack;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {line = line.substr(0, hash);}
    const auto first = line.find_first_not_of(' ');
    if (first == std::string::npos) {continue;}
    const int indent = static_cast<int>(first);
    const auto colon = line.find(':');
    if (colon == std::string::npos) {continue;}
    std::string key = line.substr(first, colon - first);
    std::string val = line.substr(colon + 1);
    val.erase(0, val.find_first_not_of(" \t"));
    val.erase(val.find_last_not_of(" \t\r") + 1);
    if (val.size() >= 2 && (val.front() == '"' || val.front() == '\'')) {val = val.substr(1, val.size() - 2);}
    while (!stack.empty() && stack.back().first >= indent) {stack.pop_back();}
    if (val.empty()) {stack.emplace_back(indent, key); continue;}
    std::string name;
    for (const auto & s : stack) {
      if (s.second == "ros__parameters" || s.first == 0) {continue;}
      name += s.second + ".";
    }
    out[name + key] = val;
  }
  return out;
}

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr << "usage: " << argv[0] << " params.yaml events.csv out.csv map.json [map2.json...]\n";
    return 2;
  }
  EstimatorParams ep; TractionParams tp; GeoParams gp;
  const auto yaml = read_yaml_flat(argv[1]);
  apply_params(make_param_table(ep, tp, gp), yaml);
  double publish_period = 0.02;
  if (yaml.count("publish.min_period")) {publish_period = std::stod(yaml.at("publish.min_period"));}
  double time_offset = 0.0;
  if (yaml.count("publish.time_offset")) {time_offset = std::stod(yaml.at("publish.time_offset"));}
  for (int i = 5; i < argc; ++i) {
    // allow overrides "name=value" after map files
    std::string a = argv[i];
    const auto eq = a.find('=');
    if (eq != std::string::npos) {
      if (a.substr(0, eq) == "publish.time_offset") {time_offset = std::stod(a.substr(eq + 1)); continue;}
      std::map<std::string, std::string> kv{{a.substr(0, eq), a.substr(eq + 1)}};
      apply_params(make_param_table(ep, tp, gp), kv);
    }
  }
  RouteMap route;
  std::vector<Landmark> landmarks;
  for (int i = 4; i < argc; ++i) {
    std::string a = argv[i];
    if (a.find('=') != std::string::npos) {continue;}
    if (a.find("stations") != std::string::npos) {landmarks = load_landmarks(a);} else {route.load_file(a);}
  }
  Estimator est(ep, tp, gp, &route);
  est.set_landmarks(landmarks);

  std::ifstream in(argv[2]);
  std::FILE * out = std::fopen(argv[3], "w");
  if (!in || !out) {std::cerr << "cannot open io files\n"; return 2;}
  std::fprintf(out, "t,t_recv,v,a,x,y,z,yaw,s_route,valid,on_map,mode,pgf,psf,pff,pgr,psr,pfr,"
    "stale_f,stale_r,mu,d,bf,br,kt,kb,var_v,var_s,notch_f,force,proc_us,init,c,lm_n\n");
  std::string line;
  std::getline(in, line);  // header
  double last_pub = -1e18;
  long n = 0;
  // emulation of the node's fixed-rate publication timer (wall clock = bag record time)
  const double timer_period = yaml.count("publish.timer_period") ? std::stod(yaml.at("publish.timer_period")) : 0.025;
  const double timer_extrap = yaml.count("publish.timer_max_extrapolation") ? std::stod(yaml.at("publish.timer_max_extrapolation")) : 0.1;
  const double wd_max = yaml.count("publish.watchdog_max_extrapolation") ? std::stod(yaml.at("publish.watchdog_max_extrapolation")) : 2.0;
  double next_tick = -1.0, last_input_recv = -1.0;
  auto write_row = [&](const EstimatorOutput & o, double t_recv, double us) {
      std::fprintf(out, "%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.3f,%.5f,%.3f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%d,%d,%.4f,%.4f,%.6f,%.6f,%.4f,%.4f,%.6f,%.5f,%.3f,%.1f,%.2f,%d,%.6f,%lu\n",
        o.t, t_recv, o.v, o.a, o.x, o.y, o.z, o.yaw, o.s_route, o.position_valid ? 1 : 0, o.on_map ? 1 : 0,
        static_cast<int>(o.mode), o.p_mode[0][0], o.p_mode[0][1], o.p_mode[0][2], o.p_mode[1][0],
        o.p_mode[1][1], o.p_mode[1][2], o.stale[0] ? 1 : 0, o.stale[1] ? 1 : 0, o.mu, o.d, o.bf, o.br,
        o.kappa_t, o.kappa_b, o.var_v, o.var_s, o.notch_f, o.force_N, us, o.initialized ? 1 : 0, o.c,
        o.landmark_updates);
      ++n;
    };
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string tok;
    double f[7] = {0, 0, 0, 0, 0, 0, 0};
    for (int k = 0; k < 7 && std::getline(ss, tok, ','); ++k) {f[k] = std::stod(tok);}
    const int kind = static_cast<int>(f[0]);
    // timer ticks that fired before this message arrived
    if (est.started() && last_input_recv >= 0.0) {
      if (next_tick < 0.0) {next_tick = last_input_recv + timer_period;}
      while (next_tick < f[1]) {
        const double idle = next_tick - last_input_recv;
        if (idle <= wd_max) {
          const double ahead = idle < 0.3 ? std::min(idle, timer_extrap) : idle;
          const double tp = est.time() + ahead;
          if (tp - last_pub >= publish_period) {
            const auto c0 = std::chrono::steady_clock::now();
            Estimator copy = est;
            if (copy.advance_to(tp)) {
              const EstimatorOutput o = copy.output_delayed(time_offset);
              last_pub = o.t;
              write_row(o, next_tick, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - c0).count());
            }
          }
        }
        next_tick += timer_period;
      }
    }
    if (kind <= 2) {last_input_recv = f[1];}
    const auto t0 = std::chrono::steady_clock::now();
    switch (kind) {
      case 0: est.on_command(f[2], f[3]); break;
      case 1: est.on_wheel(f[2], 0, f[3]); break;
      case 2: est.on_wheel(f[2], 1, f[3]); break;
      case 3: est.on_gnss_fix(f[2], true, f[3], f[4], f[5], static_cast<int>(f[6])); break;
      case 4: est.on_gnss_fix(f[2], false, f[3], f[4], f[5], static_cast<int>(f[6])); break;
      default: break;
    }
    if (kind > 2 || !est.started()) {continue;}
    if (est.time() - last_pub < publish_period) {continue;}
    const EstimatorOutput o = est.output_delayed(time_offset);
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    last_pub = o.t;
    write_row(o, f[1], us);
  }
  std::fclose(out);
  std::cerr << "published " << n << " samples, dropped " << est.output().dropped << "\n";
  return 0;
}

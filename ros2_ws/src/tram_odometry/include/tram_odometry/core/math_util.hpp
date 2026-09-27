#pragma once

#include <array>
#include <cmath>

namespace tram_odometry
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

inline double wrap_pi(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0) {a += 2.0 * kPi;}
  return a - kPi;
}

inline double clampd(double v, double lo, double hi) {return v < lo ? lo : (v > hi ? hi : v);}

/// Small fixed-size dense matrix (row-major); enough for a 6-state EKF without Eigen.
template<int R, int C>
struct Mat
{
  std::array<double, R * C> d{};
  double & operator()(int r, int c) {return d[static_cast<size_t>(r * C + c)];}
  double operator()(int r, int c) const {return d[static_cast<size_t>(r * C + c)];}
  static Mat zero() {return Mat{};}
  static Mat identity()
  {
    Mat m{};
    for (int i = 0; i < (R < C ? R : C); ++i) {m(i, i) = 1.0;}
    return m;
  }
  Mat<C, R> t() const
  {
    Mat<C, R> m{};
    for (int r = 0; r < R; ++r) {
      for (int c = 0; c < C; ++c) {m(c, r) = (*this)(r, c);}
    }
    return m;
  }
  Mat operator+(const Mat & o) const
  {
    Mat m{};
    for (size_t i = 0; i < d.size(); ++i) {m.d[i] = d[i] + o.d[i];}
    return m;
  }
  Mat operator-(const Mat & o) const
  {
    Mat m{};
    for (size_t i = 0; i < d.size(); ++i) {m.d[i] = d[i] - o.d[i];}
    return m;
  }
  Mat operator*(double s) const
  {
    Mat m{};
    for (size_t i = 0; i < d.size(); ++i) {m.d[i] = d[i] * s;}
    return m;
  }
  template<int K>
  Mat<R, K> operator*(const Mat<C, K> & o) const
  {
    Mat<R, K> m{};
    for (int r = 0; r < R; ++r) {
      for (int k = 0; k < C; ++k) {
        const double a = (*this)(r, k);
        if (a == 0.0) {continue;}
        for (int c = 0; c < K; ++c) {m(r, c) += a * o(k, c);}
      }
    }
    return m;
  }
};

}  // namespace tram_odometry

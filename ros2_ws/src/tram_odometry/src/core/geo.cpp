#include "tram_odometry/core/geo.hpp"

#include <cmath>

namespace tram_odometry
{

constexpr double kPi = 3.14159265358979323846;

void wgs84_to_utm(double lat_deg, double lon_deg, int zone, bool north, double & easting,
  double & northing)
{
  // Krueger n-series (Karney 2011), 6th order in n.
  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  constexpr double k0 = 0.9996;
  const double n = f / (2.0 - f);
  const double n2 = n * n, n3 = n2 * n, n4 = n3 * n, n5 = n4 * n, n6 = n5 * n;
  const double A = a / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);
  const double alpha[6] = {
    n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0 +
    7891.0 * n6 / 37800.0,
    13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0 -
    1983433.0 * n6 / 1935360.0,
    61.0 * n3 / 240.0 - 103.0 * n4 / 140.0 + 15061.0 * n5 / 26880.0 + 167603.0 * n6 / 181440.0,
    49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0 + 6601661.0 * n6 / 7257600.0,
    34729.0 * n5 / 80640.0 - 3418889.0 * n6 / 1995840.0,
    212378941.0 * n6 / 319334400.0};
  const double lat = lat_deg * kPi / 180.0;
  const double lon0 = (zone * 6.0 - 183.0) * kPi / 180.0;
  const double dlon = lon_deg * kPi / 180.0 - lon0;
  const double e = std::sqrt(f * (2.0 - f));
  const double t = std::sinh(std::atanh(std::sin(lat)) - e * std::atanh(e * std::sin(lat)));
  const double xi_p = std::atan2(t, std::cos(dlon));
  const double eta_p = std::atanh(std::sin(dlon) / std::sqrt(1.0 + t * t));
  double xi = xi_p, eta = eta_p;
  for (int j = 1; j <= 6; ++j) {
    xi += alpha[j - 1] * std::sin(2.0 * j * xi_p) * std::cosh(2.0 * j * eta_p);
    eta += alpha[j - 1] * std::cos(2.0 * j * xi_p) * std::sinh(2.0 * j * eta_p);
  }
  easting = 500000.0 + k0 * A * eta;
  northing = k0 * A * xi + (north ? 0.0 : 10000000.0);
}

MapPoint wgs84_to_map(double lat_deg, double lon_deg, double alt, const GeoParams & p)
{
  MapPoint m;
  double e = 0.0, n = 0.0;
  wgs84_to_utm(lat_deg, lon_deg, p.utm_zone, p.north, e, n);
  m.x = e - p.offset_x;
  m.y = n - p.offset_y;
  m.z = alt + p.offset_z;
  return m;
}

}  // namespace tram_odometry

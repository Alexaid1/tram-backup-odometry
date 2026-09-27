// WGS84 -> UTM -> local "map" frame.
// The reference frame of /localization/kinematic_state was identified offline
// (tools/fit_geo.py): map = UTM zone 37N minus (300000, 6100000) (MGRS 100 km tile),
// residual ~0.2-0.3 m on the master antenna. All values are parameters.
#pragma once

namespace tram_odometry
{

struct GeoParams
{
  int utm_zone{37};
  bool north{true};
  double offset_x{300000.0};
  double offset_y{6100000.0};
  double offset_z{-0.14};  // map_z = altitude + offset_z
};

struct MapPoint
{
  double x{0.0}, y{0.0}, z{0.0};
};

/// Transverse Mercator forward projection (Krueger series, WGS84), sub-mm within zone.
void wgs84_to_utm(double lat_deg, double lon_deg, int zone, bool north, double & easting,
  double & northing);

MapPoint wgs84_to_map(double lat_deg, double lon_deg, double alt, const GeoParams & p);

}  // namespace tram_odometry

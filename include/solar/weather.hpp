#pragma once

#include <cstdint>
#include <string>

namespace solar {

struct SolarPosition {
  // Clockwise from true north: east=90, south=180, west=270.
  float azimuth_degrees = 0.0f;
  float elevation_degrees = 0.0f;
};

struct WeatherConditions {
  double latitude = 0.0;
  double longitude = 0.0;
  float dni_w_m2 = 0.0f;
  float dhi_w_m2 = 0.0f;
  float air_temperature_c = 0.0f;
  float cloud_cover_percent = 0.0f;
  SolarPosition sun;
  std::string observation_time_utc;
};

// NOAA-style apparent solar-position calculation. Unix time is UTC seconds.
SolarPosition solar_position_utc(double latitude, double longitude,
                                 std::int64_t unix_seconds);

// Fetches the current model time step from Open-Meteo. DNI and DHI already
// include modeled cloud/atmospheric attenuation and must not be attenuated
// again using cloud_cover_percent.
WeatherConditions fetch_open_meteo_current(double latitude, double longitude);

}  // namespace solar

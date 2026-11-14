#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "solar/simulation.hpp"

namespace solar {

struct RaceConfig {
  std::filesystem::path track_path;
  std::string date;
  std::string start_local;
  std::string end_local;
  int utc_offset_seconds = -21600;
  std::string timezone = "America/Chicago";
  double start_distance_m = 0.0;
  double speed_mph = 47.5;
  double max_active_area_m2 = 6.0;
  double cell_efficiency = 0.243;
  double track_step_m = 100.0;
  double position_step_m = 0.128;
  double angle_step_degrees = 15.0;
};

struct TrackPoint {
  double distance_m = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double elevation_m = 0.0;
};

struct Track {
  std::vector<TrackPoint> points;
  // length_m preserves the CSV's horizontal distance axis; surface distance
  // adds the elevation component and is used for timed speed/positioning.
  double length_m = 0.0;
  double surface_length_m = 0.0;
  std::vector<double> surface_distances_m;
  std::string distance_origin = "track reference origin; not a verified start/finish line";
};

struct HourlyWeather {
  std::int64_t unix_seconds = 0;
  double dni_w_m2 = 0.0;
  double dhi_w_m2 = 0.0;
};

struct RaceSample {
  Sun sun;
  double duration_seconds = 0.0;
  double elapsed_seconds = 0.0;
  double distance_m = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double heading_degrees = 0.0;
  double pitch_degrees = 0.0;
};

struct RaceRun {
  std::vector<RaceSample> samples;
  double duration_seconds = 0.0;
  double distance_m = 0.0;
  double laps = 0.0;
  std::string weather_source;
};

RaceConfig load_race_config(const std::filesystem::path& path);
void save_race_config(const std::filesystem::path& path,
                      const RaceConfig& config);
Track load_track(const std::filesystem::path& path);

// Display-only local metre coordinates: east, north, elevation relative to
// the first sample. Uses WGS84 curvature at the reference latitude; the race
// calculations continue to use the original geographic coordinates.
std::vector<Vec3> track_centerline_vertices(const Track& track);

// Offline/deterministic builder. Hourly DNI/DHI values are linearly
// interpolated between their UTC timestamps; caller-supplied timestamps denote
// the time represented by the values (midpoints for preceding-hour means).
// Coverage must bracket every sample midpoint. Radiation is not re-attenuated.
RaceRun build_race_run(const RaceConfig& config, const Track& track,
                       const std::vector<HourlyWeather>& weather);

// Fetches hourly radiation for the configured calendar date from Open-Meteo,
// then delegates to the deterministic builder.
RaceRun build_race_run(const RaceConfig& config, const Track& track);

}  // namespace solar

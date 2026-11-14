#include "solar/race.hpp"

#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
int failures = 0;
void require(bool condition, std::string_view message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  ++failures;
}
bool close(double a, double b, double tolerance = 1.0e-5) {
  return std::abs(a - b) <= tolerance;
}
struct TemporaryTrack {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("solar-track-audit-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".csv");
  ~TemporaryTrack() { std::error_code error; std::filesystem::remove(path, error); }
};
void malformed_track_is_rejected(const std::string& data, const char* message) {
  TemporaryTrack fixture;
  { std::ofstream output(fixture.path); output << data; }
  bool rejected = false;
  try { (void)solar::load_track(fixture.path); }
  catch (const std::runtime_error&) { rejected = true; }
  require(rejected, message);
}
}  // namespace

int main() {
  malformed_track_is_rejected("s,lat,lon,z\n0,38,-92,100\n100,38.001,-92,101\n200,38,-92,102\n",
                              "track elevation must close with its coordinates");
  malformed_track_is_rejected("s,lat,lon,z\n0,38,-92,100\n100bad,38.001,-92,101\n200,38,-92,100\n",
                              "CSV numeric fields must not accept trailing garbage");
  solar::RaceConfig config;
  config.track_path = "synthetic.csv";
  config.date = "2026-11-07";
  config.start_local = "09:00";
  config.end_local = "09:05";
  config.utc_offset_seconds = -21600;
  config.start_distance_m = 50.0;
  config.track_step_m = 100.0;

  solar::Track track;
  track.points = {{0.0, 38.0, -92.0, 100.0},
                  {100.0, 38.0009, -92.0, 102.0},
                  {200.0, 38.0009, -91.9989, 103.0},
                  {300.0, 38.0, -91.9989, 101.0},
                  {400.0, 38.0, -92.0, 100.0}};
  track.length_m = 400.0;

  // 09:00 CST is 15:00 UTC. Samples are linearly interpolated between
  // hourly values rather than treated as a zero-order hold.
  const std::vector<solar::HourlyWeather> weather{
      {1794060000LL, 200.0, 40.0}, {1794067200LL, 800.0, 100.0},
      {1794074400LL, 600.0, 80.0}};
  const solar::RaceRun run = solar::build_race_run(config, track, weather);
  require(run.samples.size() > 1, "race is discretized into timed samples");
  require(close(run.duration_seconds, 300.0), "run duration matches clock window");
  require(close(run.distance_m, 47.5 * 0.44704 * 300.0),
          "race distance uses configured mph");
  double surface_length = 0.0;
  for (std::size_t i = 1; i < track.points.size(); ++i) {
    surface_length += std::hypot(track.points[i].distance_m - track.points[i - 1].distance_m,
                                 track.points[i].elevation_m - track.points[i - 1].elevation_m);
  }
  require(close(run.laps, run.distance_m / surface_length),
          "fractional lap count uses 3D road distance");
  require(close(run.samples.front().distance_m,
                std::fmod(config.start_distance_m +
                              run.samples.front().duration_seconds * 0.5 * 47.5 * 0.44704,
                          surface_length),
                1e-5),
          "configured start station is applied before race travel and wrapped per lap");
  double duration_sum = 0.0;
  double previous_elapsed = -1.0;
  for (const auto& sample : run.samples) {
    duration_sum += sample.duration_seconds;
    require(sample.duration_seconds > 0.0 && sample.duration_seconds <=
                config.track_step_m / (config.speed_mph * 0.44704) + 1e-9,
            "sample duration does not exceed track distance step");
    require(sample.elapsed_seconds > previous_elapsed,
            "sample elapsed times increase strictly");
    previous_elapsed = sample.elapsed_seconds;
    require(close(solar::length(sample.sun.direction_to_sun), 1.0, 1e-4),
            "car-local sun vector is normalized");
    require(close(solar::length(sample.sun.world_up), 1.0, 1e-4),
            "geographic vertical is normalized in car frame");
    require(close(sample.sun.world_up.y,
                  std::sin(sample.pitch_degrees * solar::kPi / 180.0), 1e-5),
            "uphill geographic vertical has positive forward component");
    const double heading = sample.heading_degrees * solar::kPi / 180.0;
    const double pitch = sample.pitch_degrees * solar::kPi / 180.0;
    const solar::Vec3 right{static_cast<float>(std::cos(heading)),
                            static_cast<float>(-std::sin(heading)), 0.0f};
    const solar::Vec3 forward{static_cast<float>(std::sin(heading) * std::cos(pitch)),
                              static_cast<float>(std::cos(heading) * std::cos(pitch)),
                              static_cast<float>(std::sin(pitch))};
    const solar::Vec3 overhead_local{
        solar::dot({0.0f, 0.0f, 1.0f}, right),
        solar::dot({0.0f, 0.0f, 1.0f}, forward),
        solar::dot({0.0f, 0.0f, 1.0f}, solar::normalized(solar::cross(right, forward)))};
    require(close(overhead_local.x, 0.0, 1e-5) &&
                close(overhead_local.y, sample.sun.world_up.y, 1e-5) &&
                close(overhead_local.z, sample.sun.world_up.z, 1e-5),
            "overhead sun on an uphill road matches geographic vertical in car frame");
    require(sample.sun.dni_w_m2 >= 200.0f && sample.sun.dni_w_m2 <= 800.0f,
            "hourly DNI is interpolated within input range");
    require(sample.sun.dhi_w_m2 >= 40.0f && sample.sun.dhi_w_m2 <= 100.0f,
            "hourly DHI is interpolated within input range");
    require(sample.pitch_degrees > 0.0 || sample.pitch_degrees < 0.0,
            "track pitch follows local elevation grade");
  }
  require(close(duration_sum, 300.0), "partial final sample preserves exact total duration");
  require(run.samples.back().elapsed_seconds + run.samples.back().duration_seconds <=
              run.duration_seconds + 1e-9,
          "no sample extends past race end");

  bool rejected = false;
  try {
    const std::vector<solar::HourlyWeather> insufficient{{1794063600LL, 1.0, 1.0},
                                                          {1794063720LL, 1.0, 1.0}};
    (void)solar::build_race_run(config, track, insufficient);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "weather must bracket every sample midpoint");

  const solar::RaceConfig configured = solar::load_race_config("config/race.json");
  const solar::Track oir = solar::load_track(configured.track_path);
  require(configured.date == "2026-11-07" && configured.utc_offset_seconds == -21600,
          "race config keeps the requested date and explicit CST offset");
  require(oir.points.size() == 1233 && close(oir.length_m, 6159.146, 0.02),
          "track loader preserves the supplied 5 m centerline and distance origin");
  require(oir.surface_length_m > oir.length_m,
          "surface distance includes the centerline elevation profile");
  const auto centerline = solar::track_centerline_vertices(oir);
  require(centerline.size() == oir.points.size(), "display preserves all centerline samples");
  require(solar::length(centerline.front()) == 0 && solar::length(centerline.back()) < 0.001f,
          "display centerline starts at the local origin and closes");
  solar::Bounds display_bounds;
  for (const auto& vertex : centerline) display_bounds.include(vertex);
  require(display_bounds.extent().x > 100 && display_bounds.extent().y > 100,
          "geographic centerline is projected to metres rather than degrees");
  double min_elevation = oir.points.front().elevation_m;
  double max_elevation = min_elevation;
  for (const auto& point : oir.points) {
    min_elevation = std::min(min_elevation, point.elevation_m);
    max_elevation = std::max(max_elevation, point.elevation_m);
  }
  require(close(display_bounds.extent().z, max_elevation - min_elevation, 0.001),
          "track display preserves true elevation differences without exaggeration");
  const auto full_day = solar::build_race_run(configured, oir,
      {{0, 800, 100}, {4102444800LL, 800, 100}});
  double full_day_seconds = 0;
  for (const auto& sample : full_day.samples) full_day_seconds += sample.duration_seconds;
  require(close(full_day.duration_seconds, 8 * 3600) && close(full_day_seconds, 8 * 3600),
          "default race preserves the complete eight-hour window");
  require(full_day.samples.size() == 6116 && full_day.laps > 90,
          "default 47.5 mph run repeatedly samples the supplied track");

  if (failures != 0) return 1;
  std::cout << "Race tests passed\n";
  return 0;
}

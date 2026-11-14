#include "solar/race.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <cpr/cpr.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "solar/weather.hpp"

namespace solar {
namespace {

constexpr double kMphToMps = 0.44704;

std::int64_t days_from_civil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
  const unsigned adjusted_month = month > 2 ? month - 3U : month + 9U;
  const unsigned day_of_year = (153U * adjusted_month + 2U) / 5U + day - 1U;
  const unsigned day_of_era = year_of_era * 365U + year_of_era / 4U -
                              year_of_era / 100U + day_of_year;
  return static_cast<std::int64_t>(era) * 146097LL + day_of_era - 719468LL;
}

bool leap_year(int year) {
  return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

std::int64_t parse_local_time(const RaceConfig& config,
                              const std::string& time) {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  char d1 = 0, d2 = 0, sep = 0, c1 = 0, c2 = 0;
  std::istringstream input(config.date + "T" + time);
  input >> year >> d1 >> month >> d2 >> day >> sep >> hour >> c1 >> minute;
  const bool parsed_base = static_cast<bool>(input);
  if (parsed_base && time.size() == 8) input >> c2 >> second;
  static constexpr int month_days[] = {0, 31, 28, 31, 30, 31, 30,
                                        31, 31, 30, 31, 30, 31};
  const int max_day = month == 2 && leap_year(year)
                          ? 29
                          : (month >= 1 && month <= 12 ? month_days[month] : 0);
  if (!parsed_base || (time.size() == 8 && (!input || c2 != ':')) ||
      d1 != '-' || d2 != '-' || sep != 'T' || c1 != ':' ||
      (time.size() != 5 && time.size() != 8) || day < 1 || day > max_day ||
      hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 ||
      second > 59) {
    throw std::runtime_error("Invalid race date/time: " + config.date + " " +
                             time);
  }
  return days_from_civil(year, static_cast<unsigned>(month),
                         static_cast<unsigned>(day)) *
             86400LL +
         hour * 3600LL + minute * 60LL + second - config.utc_offset_seconds;
}

std::int64_t parse_utc_time(const std::string& text) {
  if (text.size() < 16) throw std::runtime_error("Invalid UTC timestamp: " + text);
  RaceConfig config;
  config.date = text.substr(0, 10);
  config.utc_offset_seconds = 0;
  return parse_local_time(config, text.substr(11, 5) +
                                      (text.size() >= 19 ? text.substr(16, 3)
                                                        : ""));
}

void validate_config(const RaceConfig& config) {
  if (config.date.size() != 10 || config.timezone.empty() || config.track_path.empty())
    throw std::runtime_error("Race config requires date, timezone, and track_path");
  (void)parse_local_time(config, config.start_local);
  (void)parse_local_time(config, config.end_local);
  const auto start = parse_local_time(config, config.start_local);
  const auto end = parse_local_time(config, config.end_local);
  if (end <= start) throw std::runtime_error("Race end must follow race start");
  if (std::abs(config.utc_offset_seconds) > 14 * 3600)
    throw std::runtime_error("Race UTC offset must be within +/-14 hours");
  const auto positive_finite = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive_finite(config.speed_mph) ||
      !std::isfinite(config.start_distance_m) || config.start_distance_m < 0.0 ||
      !positive_finite(config.max_active_area_m2) ||
      !positive_finite(config.track_step_m) ||
      !positive_finite(config.position_step_m) ||
      !positive_finite(config.angle_step_degrees) ||
      !std::isfinite(config.cell_efficiency) || config.cell_efficiency <= 0.0 ||
      config.cell_efficiency > 1.0 || config.angle_step_degrees > 180.0) {
    throw std::runtime_error("Race config contains an out-of-range numeric value");
  }
}

double number(const nlohmann::json& object, const char* key, double fallback) {
  const auto it = object.find(key);
  if (it == object.end()) return fallback;
  if (!it->is_number())
    throw std::runtime_error(std::string("Race config field must be numeric: ") + key);
  const double value = it->get<double>();
  if (!std::isfinite(value))
    throw std::runtime_error(std::string("Race config field must be finite: ") + key);
  return value;
}

std::string required_string(const nlohmann::json& object, const char* key) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_string())
    throw std::runtime_error(std::string("Race config missing string field: ") + key);
  return it->get<std::string>();
}

struct Position {
  double latitude;
  double longitude;
  double elevation;
};

std::vector<double> make_surface_stations(const Track& track) {
  if (track.surface_distances_m.size() == track.points.size())
    return track.surface_distances_m;
  std::vector<double> stations(track.points.size(), 0.0);
  for (std::size_t i = 1; i < track.points.size(); ++i) {
    const double horizontal = track.points[i].distance_m - track.points[i - 1].distance_m;
    const double vertical = track.points[i].elevation_m - track.points[i - 1].elevation_m;
    stations[i] = stations[i - 1] + std::hypot(horizontal, vertical);
  }
  return stations;
}

Position interpolate_track(const Track& track, const std::vector<double>& stations,
                           double surface_distance, double surface_length) {
  surface_distance = std::fmod(surface_distance, surface_length);
  if (surface_distance < 0.0) surface_distance += surface_length;
  const auto upper = std::upper_bound(
      stations.begin(), stations.end(), surface_distance);
  const std::size_t i1 = upper == stations.end()
                             ? stations.size() - 1
                             : static_cast<std::size_t>(upper - stations.begin());
  const std::size_t i0 = i1 == 0 ? 0 : i1 - 1;
  const auto& a = track.points[i0];
  const auto& b = track.points[i1];
  const double span = stations[i1] - stations[i0];
  const double t = span > 0.0 ? std::clamp((surface_distance - stations[i0]) / span, 0.0, 1.0)
                              : 0.0;
  return {a.latitude + (b.latitude - a.latitude) * t,
          a.longitude + (b.longitude - a.longitude) * t,
          a.elevation_m + (b.elevation_m - a.elevation_m) * t};
}

double heading_degrees(const Track& track, const std::vector<double>& stations,
                       double distance, double surface_length) {
  const Position a = interpolate_track(track, stations, distance - 1.0, surface_length);
  const Position b = interpolate_track(track, stations, distance + 1.0, surface_length);
  const double lat0 = (a.latitude + b.latitude) * 0.5 * std::numbers::pi / 180.0;
  const double east = (b.longitude - a.longitude) * std::cos(lat0);
  const double north = b.latitude - a.latitude;
  double heading = std::atan2(east, north) * 180.0 / std::numbers::pi;
  if (heading < 0.0) heading += 360.0;
  return heading;
}

double pitch_degrees(const Track& track, const std::vector<double>& stations,
                     double distance, double surface_length) {
  const Position a = interpolate_track(track, stations, distance - 1.0, surface_length);
  const Position b = interpolate_track(track, stations, distance + 1.0, surface_length);
  return std::asin(std::clamp((b.elevation - a.elevation) / 2.0, -1.0, 1.0)) * 180.0 /
         std::numbers::pi;
}

HourlyWeather interpolate_weather(const std::vector<HourlyWeather>& weather,
                                   std::int64_t time) {
  if (time == weather.back().unix_seconds) return weather.back();
  const auto upper = std::upper_bound(
      weather.begin(), weather.end(), time,
      [](std::int64_t value, const HourlyWeather& item) {
        return value < item.unix_seconds;
      });
  if (upper == weather.begin() || upper == weather.end())
    throw std::runtime_error("Hourly weather does not bracket race sample time");
  const auto& b = *upper;
  const auto& a = *(upper - 1);
  const double fraction = static_cast<double>(time - a.unix_seconds) /
                          static_cast<double>(b.unix_seconds - a.unix_seconds);
  return {time, a.dni_w_m2 + (b.dni_w_m2 - a.dni_w_m2) * fraction,
          a.dhi_w_m2 + (b.dhi_w_m2 - a.dhi_w_m2) * fraction};
}

}  // namespace

RaceConfig load_race_config(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open race config: " + path.string());
  nlohmann::json json;
  try {
    input >> json;
  } catch (const nlohmann::json::exception& error) {
    throw std::runtime_error(std::string("Invalid race config JSON: ") + error.what());
  }
  if (!json.is_object()) throw std::runtime_error("Race config must be a JSON object");
  RaceConfig config;
  config.track_path = required_string(json, "track_path");
  if (config.track_path.is_relative())
    config.track_path = path.parent_path() / config.track_path;
  config.date = required_string(json, "date");
  config.start_local = required_string(json, "start_local");
  config.end_local = required_string(json, "end_local");
  config.timezone = required_string(json, "timezone");
  config.start_distance_m = number(json, "start_distance_m", config.start_distance_m);
  const double offset = number(json, "utc_offset_seconds", -21600);
  if (std::floor(offset) != offset || offset < -50400.0 || offset > 50400.0)
    throw std::runtime_error("Race UTC offset must be an integer within +/-14 hours");
  config.utc_offset_seconds = static_cast<int>(offset);
  config.speed_mph = number(json, "speed_mph", config.speed_mph);
  config.max_active_area_m2 = number(json, "max_active_area_m2", config.max_active_area_m2);
  config.cell_efficiency = number(json, "cell_efficiency", config.cell_efficiency);
  config.track_step_m = number(json, "track_step_m", config.track_step_m);
  config.position_step_m = number(json, "position_step_m", config.position_step_m);
  config.angle_step_degrees = number(json, "angle_step_degrees", config.angle_step_degrees);
  validate_config(config);
  return config;
}

void save_race_config(const std::filesystem::path& path,
                      const RaceConfig& config) {
  validate_config(config);
  const std::filesystem::path output_path = path.is_absolute()
                                                ? path
                                                : std::filesystem::absolute(path);
  const std::filesystem::path config_directory =
      output_path.parent_path().empty() ? std::filesystem::current_path()
                                        : output_path.parent_path();
  const std::filesystem::path track_absolute = config.track_path.is_absolute()
                                                  ? config.track_path.lexically_normal()
                                                  : std::filesystem::absolute(config.track_path).lexically_normal();
  std::filesystem::path relative_track = track_absolute.lexically_relative(
      config_directory.lexically_normal());
  if (relative_track.empty()) relative_track = config.track_path.filename();

  nlohmann::json json{{"track_path", relative_track.generic_string()},
                      {"date", config.date},
                      {"start_local", config.start_local},
                      {"end_local", config.end_local},
                      {"utc_offset_seconds", config.utc_offset_seconds},
                      {"timezone", config.timezone},
                      {"start_distance_m", config.start_distance_m},
                      {"speed_mph", config.speed_mph},
                      {"max_active_area_m2", config.max_active_area_m2},
                      {"cell_efficiency", config.cell_efficiency},
                      {"track_step_m", config.track_step_m},
                      {"position_step_m", config.position_step_m},
                      {"angle_step_degrees", config.angle_step_degrees}};
  std::ofstream output(output_path);
  if (!output)
    throw std::runtime_error("Cannot write race config: " + output_path.string());
  output << json.dump(2) << '\n';
  if (!output)
    throw std::runtime_error("Failed writing race config: " + output_path.string());
}

Track load_track(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open track CSV: " + path.string());
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("Track CSV is empty");
  Track track;
  std::size_t line_number = 1;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) continue;
    std::istringstream row(line);
    std::string fields[4];
    for (int i = 0; i < 4; ++i) {
      if (!std::getline(row, fields[i], ','))
        throw std::runtime_error("Malformed track CSV row " + std::to_string(line_number));
    }
    try {
      const auto parse_field = [&](const std::string& field) {
        std::size_t used = 0;
        const double value = std::stod(field, &used);
        if (field.find_first_not_of(" \t\r", used) != std::string::npos)
          throw std::invalid_argument("Trailing CSV field data");
        return value;
      };
      TrackPoint point{parse_field(fields[0]), parse_field(fields[1]),
                       parse_field(fields[2]), parse_field(fields[3])};
      if (!std::isfinite(point.distance_m) || !std::isfinite(point.latitude) ||
          !std::isfinite(point.longitude) || !std::isfinite(point.elevation_m) ||
          point.latitude < -90.0 || point.latitude > 90.0 ||
          point.longitude < -180.0 || point.longitude > 180.0 ||
          (!track.points.empty() && point.distance_m <= track.points.back().distance_m))
        throw std::runtime_error("Invalid or non-increasing track CSV row " +
                                 std::to_string(line_number));
      track.points.push_back(point);
    } catch (const std::invalid_argument&) {
      throw std::runtime_error("Invalid number in track CSV row " + std::to_string(line_number));
    } catch (const std::out_of_range&) {
      throw std::runtime_error("Out-of-range number in track CSV row " + std::to_string(line_number));
    }
  }
  if (track.points.size() < 3 || std::abs(track.points.front().distance_m) > 1e-6 ||
      track.points.front().latitude != track.points.back().latitude ||
      track.points.front().longitude != track.points.back().longitude ||
      std::abs(track.points.front().elevation_m - track.points.back().elevation_m) > 1.0e-6) {
    throw std::runtime_error("Track must start at distance zero and end with a repeated closure point");
  }
  track.length_m = track.points.back().distance_m;
  track.surface_distances_m = make_surface_stations(track);
  track.surface_length_m = track.surface_distances_m.back();
  return track;
}

std::vector<Vec3> track_centerline_vertices(const Track& track) {
  if (track.points.size() < 2)
    throw std::invalid_argument("Track display needs at least two points");
  constexpr double radians = std::numbers::pi / 180.0;
  constexpr double a = 6378137.0;
  constexpr double flattening = 1.0 / 298.257223563;
  constexpr double e2 = flattening * (2.0 - flattening);
  const auto& origin = track.points.front();
  const double latitude = origin.latitude * radians;
  const double denominator = 1.0 - e2 * std::sin(latitude) * std::sin(latitude);
  const double north_radius = a * (1.0 - e2) / std::pow(denominator, 1.5);
  const double east_radius = a / std::sqrt(denominator) * std::cos(latitude);
  std::vector<Vec3> vertices;
  vertices.reserve(track.points.size());
  for (const auto& point : track.points) {
    const double east = (point.longitude - origin.longitude) * radians * east_radius;
    const double north = (point.latitude - origin.latitude) * radians * north_radius;
    const double up = point.elevation_m - origin.elevation_m;
    if (!std::isfinite(east) || !std::isfinite(north) || !std::isfinite(up))
      throw std::invalid_argument("Track display coordinates must be finite");
    vertices.emplace_back(static_cast<float>(east), static_cast<float>(north), static_cast<float>(up));
  }
  return vertices;
}

RaceRun build_race_run(const RaceConfig& config, const Track& track,
                       const std::vector<HourlyWeather>& weather) {
  validate_config(config);
  if (track.points.size() < 3 || !std::isfinite(track.length_m) || track.length_m <= 0.0)
    throw std::runtime_error("Race requires a valid closed track");
  const std::vector<double> stations = make_surface_stations(track);
  if (stations.size() != track.points.size() ||
      std::abs(track.points.front().distance_m) > 1e-6 ||
      std::abs(track.points.back().distance_m - track.length_m) > 1e-6 ||
      track.points.front().latitude != track.points.back().latitude ||
      track.points.front().longitude != track.points.back().longitude)
    throw std::runtime_error("Race track must be an ordered closed loop starting at distance zero");
  for (std::size_t i = 0; i < track.points.size(); ++i) {
    const auto& point = track.points[i];
    if (!std::isfinite(point.distance_m) || !std::isfinite(point.latitude) ||
        !std::isfinite(point.longitude) || !std::isfinite(point.elevation_m) ||
        point.latitude < -90.0 || point.latitude > 90.0 ||
        point.longitude < -180.0 || point.longitude > 180.0 ||
        !std::isfinite(stations[i]) ||
        (i > 0 && (point.distance_m <= track.points[i - 1].distance_m ||
                   stations[i] <= stations[i - 1])))
      throw std::runtime_error("Race track contains invalid or unordered samples");
  }
  const double surface_length = stations.back();
  if (!std::isfinite(surface_length) || surface_length <= 0.0)
    throw std::runtime_error("Race track has invalid surface distance");
  if (weather.size() < 2) throw std::runtime_error("At least two hourly weather points are required");
  for (std::size_t i = 0; i < weather.size(); ++i) {
    if (!std::isfinite(weather[i].dni_w_m2) || !std::isfinite(weather[i].dhi_w_m2) ||
        weather[i].dni_w_m2 < 0.0 || weather[i].dhi_w_m2 < 0.0 ||
        (i > 0 && weather[i].unix_seconds <= weather[i - 1].unix_seconds))
      throw std::runtime_error("Hourly weather must be finite, nonnegative, and strictly ordered");
  }

  const std::int64_t start = parse_local_time(config, config.start_local);
  const std::int64_t end = parse_local_time(config, config.end_local);
  const double duration = static_cast<double>(end - start);
  const double speed = config.speed_mph * kMphToMps;
  const double step_duration = config.track_step_m / speed;
  const double count_value = std::ceil(duration / step_duration);
  if (!std::isfinite(step_duration) || step_duration <= 0.0 ||
      !std::isfinite(count_value) ||
      count_value > static_cast<double>(std::numeric_limits<std::size_t>::max()))
    throw std::runtime_error("Race discretization produces an invalid sample count");
  const std::size_t count = static_cast<std::size_t>(count_value);
  RaceRun run;
  run.duration_seconds = duration;
  run.distance_m = speed * duration;
  run.laps = run.distance_m / surface_length;
  run.weather_source = "caller-supplied hourly Open-Meteo-compatible data (linear UTC interpolation)";
  run.samples.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double elapsed = static_cast<double>(i) * step_duration;
    const double dt = std::min(step_duration, duration - elapsed);
    const double midpoint_elapsed = elapsed + dt * 0.5;
    const double traveled_midpoint = speed * midpoint_elapsed;
    const std::int64_t midpoint_time = start + static_cast<std::int64_t>(std::llround(midpoint_elapsed));
    const HourlyWeather irradiance = interpolate_weather(weather, midpoint_time);
    double station_distance = std::fmod(config.start_distance_m + traveled_midpoint,
                                        surface_length);
    if (station_distance < 0.0) station_distance += surface_length;
    const Position position = interpolate_track(track, stations, station_distance, surface_length);
    const double heading = heading_degrees(track, stations, station_distance, surface_length);
    const double pitch = pitch_degrees(track, stations, station_distance, surface_length);
    const SolarPosition solar_position = solar_position_utc(
        position.latitude, position.longitude, midpoint_time);
    const Vec3 enu = sun_direction_from_azimuth_elevation(
        solar_position.azimuth_degrees, solar_position.elevation_degrees);
    const double h = heading * std::numbers::pi / 180.0;
    const double p = pitch * std::numbers::pi / 180.0;
    const Vec3 right{static_cast<float>(std::cos(h)),
                     static_cast<float>(-std::sin(h)), 0.0f};
    const Vec3 forward{static_cast<float>(std::sin(h) * std::cos(p)),
                       static_cast<float>(std::cos(h) * std::cos(p)),
                       static_cast<float>(std::sin(p))};
    const Vec3 local_up = normalized(cross(right, forward));
    const Vec3 local_sun{dot(enu, right), dot(enu, forward), dot(enu, local_up)};
    RaceSample sample;
    sample.sun.direction_to_sun = normalized(local_sun);
    sample.sun.world_up = {0.0f, static_cast<float>(std::sin(p)),
                           static_cast<float>(std::cos(p))};
    sample.sun.dni_w_m2 = static_cast<float>(irradiance.dni_w_m2);
    sample.sun.dhi_w_m2 = static_cast<float>(irradiance.dhi_w_m2);
    sample.duration_seconds = dt;
    sample.elapsed_seconds = elapsed;
    sample.distance_m = station_distance;
    sample.latitude = position.latitude;
    sample.longitude = position.longitude;
    sample.heading_degrees = heading;
    sample.pitch_degrees = pitch;
    run.samples.push_back(sample);
  }
  return run;
}

RaceRun build_race_run(const RaceConfig& config, const Track& track) {
  validate_config(config);
  // Explicit fixed offset determines the race interval; the named timezone is
  // retained as descriptive metadata and never depends on the host timezone.
  const auto start = parse_local_time(config, config.start_local);
  const auto end = parse_local_time(config, config.end_local);
  const auto iso_date = [](std::int64_t seconds) {
    const std::int64_t days = seconds >= 0 ? seconds / 86400 : (seconds - 86399) / 86400;
    // Convert epoch day back to Gregorian civil date.
    std::int64_t z = days + 719468;
    const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int year = static_cast<int>(yoe) + static_cast<int>(era * 400);
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned day = doy - (153 * mp + 2) / 5 + 1;
    const unsigned month = mp < 10 ? mp + 3 : mp - 9;
    year += month <= 2;
    char buffer[11];
    std::snprintf(buffer, sizeof(buffer), "%04d-%02u-%02u", year, month, day);
    return std::string(buffer);
  };
  const std::string from_date = iso_date(start - 3600LL);
  const std::string through_date = iso_date(end + 3600LL);
  const std::int64_t now = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  const std::string endpoint = start >= now - 90LL * 86400LL
      ? "https://api.open-meteo.com/v1/forecast"
      : "https://archive-api.open-meteo.com/v1/archive";
  const cpr::Response response = cpr::Get(
      cpr::Url{endpoint},
      cpr::Parameters{{"latitude", std::to_string(track.points.front().latitude)},
                      {"longitude", std::to_string(track.points.front().longitude)},
                      {"hourly", "direct_normal_irradiance,diffuse_radiation"},
                      {"start_date", from_date}, {"end_date", through_date},
                      {"timezone", "GMT"}},
      cpr::Header{{"Accept", "application/json"}}, cpr::Timeout{20000});
  if (response.error.code != cpr::ErrorCode::OK)
    throw std::runtime_error("Open-Meteo network error: " + response.error.message);
  if (response.status_code != 200)
    throw std::runtime_error("Open-Meteo HTTP " + std::to_string(response.status_code) +
                             ": " + response.text.substr(0, 240));
  const auto payload = nlohmann::json::parse(response.text);
  if (!payload.contains("hourly") || !payload["hourly"].is_object())
    throw std::runtime_error("Open-Meteo response is missing hourly weather");
  const auto& hourly = payload["hourly"];
  for (const char* key : {"time", "direct_normal_irradiance", "diffuse_radiation"}) {
    if (!hourly.contains(key) || !hourly[key].is_array())
      throw std::runtime_error(std::string("Open-Meteo hourly response missing ") + key);
  }
  const auto& times = hourly["time"];
  const auto& dni = hourly["direct_normal_irradiance"];
  const auto& dhi = hourly["diffuse_radiation"];
  if (times.size() != dni.size() || times.size() != dhi.size())
    throw std::runtime_error("Open-Meteo hourly arrays have inconsistent lengths");
  std::vector<HourlyWeather> weather;
  weather.reserve(times.size());
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (!times[i].is_string() || dni[i].is_null() || dhi[i].is_null() ||
        !dni[i].is_number() || !dhi[i].is_number())
      throw std::runtime_error("Open-Meteo hourly response contains missing radiation data");
    // Open-Meteo hourly DNI/DHI are means over the preceding hour. Associate
    // each mean with its interval midpoint for schedule interpolation.
    weather.push_back({parse_utc_time(times[i].get<std::string>()) - 1800LL,
                       dni[i].get<double>(), dhi[i].get<double>()});
  }
  RaceRun run = build_race_run(config, track, weather);
  run.weather_source = endpoint + " hourly DNI/DHI for " + from_date + " through " + through_date;
  return run;
}

}  // namespace solar

#include "solar/weather.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <cpr/cpr.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

namespace solar {
namespace {

double radians(double degrees) {
  return degrees * std::numbers::pi_v<double> / 180.0;
}
double degrees(double radians_value) {
  return radians_value * 180.0 / std::numbers::pi_v<double>;
}

double wrap_degrees(double value) {
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

// Days since 1970-01-01. This civil-calendar conversion is independent of the
// host timezone and avoids platform-specific timegm/_mkgmtime behavior.
std::int64_t days_from_civil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era =
      static_cast<unsigned>(year - era * 400);  // [0, 399]
  const unsigned adjusted_month = month > 2 ? month - 3U : month + 9U;
  const unsigned day_of_year =
      (153U * adjusted_month + 2U) / 5U + day - 1U;
  const unsigned day_of_era =
      year_of_era * 365U + year_of_era / 4U - year_of_era / 100U +
      day_of_year;
  return static_cast<std::int64_t>(era) * 146097LL +
         static_cast<std::int64_t>(day_of_era) - 719468LL;
}

std::int64_t parse_open_meteo_utc(const std::string& text) {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  char dash1 = 0, dash2 = 0, separator = 0, colon1 = 0, colon2 = 0;
  std::istringstream input(text);
  input >> year >> dash1 >> month >> dash2 >> day >> separator >> hour >>
      colon1 >> minute;
  if (!input || dash1 != '-' || dash2 != '-' ||
      (separator != 'T' && separator != ' ') || colon1 != ':') {
    throw std::runtime_error("Open-Meteo returned an invalid UTC time: " + text);
  }
  if (input.peek() == ':') {
    input >> colon2 >> second;
    if (!input || colon2 != ':')
      throw std::runtime_error("Open-Meteo returned an invalid UTC time: " +
                               text);
  }
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 ||
      hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) {
    throw std::runtime_error("Open-Meteo returned an out-of-range UTC time: " +
                             text);
  }
  return days_from_civil(year, static_cast<unsigned>(month),
                         static_cast<unsigned>(day)) *
             86400LL +
         hour * 3600LL + minute * 60LL + second;
}

float required_number(const nlohmann::json& object, const char* key) {
  const auto found = object.find(key);
  if (found == object.end() || !found->is_number())
    throw std::runtime_error(std::string("Open-Meteo response is missing '") +
                             key + "'");
  const double value = found->get<double>();
  if (!std::isfinite(value))
    throw std::runtime_error(std::string("Open-Meteo returned invalid '") + key +
                             "'");
  return static_cast<float>(value);
}

}  // namespace

SolarPosition solar_position_utc(double latitude, double longitude,
                                 std::int64_t unix_seconds) {
  if (!std::isfinite(latitude) || latitude < -90.0 || latitude > 90.0 ||
      !std::isfinite(longitude) || longitude < -180.0 || longitude > 180.0) {
    throw std::runtime_error("Latitude/longitude are outside valid ranges");
  }

  const double julian_day =
      static_cast<double>(unix_seconds) / 86400.0 + 2440587.5;
  const double centuries = (julian_day - 2451545.0) / 36525.0;
  const double mean_longitude = wrap_degrees(
      280.46646 + centuries * (36000.76983 + centuries * 0.0003032));
  const double mean_anomaly =
      357.52911 + centuries * (35999.05029 - 0.0001537 * centuries);
  const double eccentricity =
      0.016708634 - centuries * (0.000042037 + 0.0000001267 * centuries);
  const double anomaly_radians = radians(mean_anomaly);
  const double equation_of_center =
      std::sin(anomaly_radians) *
          (1.914602 - centuries * (0.004817 + 0.000014 * centuries)) +
      std::sin(2.0 * anomaly_radians) * (0.019993 - 0.000101 * centuries) +
      std::sin(3.0 * anomaly_radians) * 0.000289;
  const double true_longitude = mean_longitude + equation_of_center;
  const double omega = 125.04 - 1934.136 * centuries;
  const double apparent_longitude =
      true_longitude - 0.00569 - 0.00478 * std::sin(radians(omega));
  const double mean_obliquity =
      23.0 + (26.0 +
              (21.448 - centuries *
                            (46.815 + centuries * (0.00059 -
                                                    centuries * 0.001813))) /
                  60.0) /
                 60.0;
  const double obliquity =
      mean_obliquity + 0.00256 * std::cos(radians(omega));
  const double declination = std::asin(
      std::sin(radians(obliquity)) * std::sin(radians(apparent_longitude)));

  const double y = std::tan(radians(obliquity) / 2.0);
  const double y_squared = y * y;
  const double longitude_radians = radians(mean_longitude);
  const double equation_of_time =
      4.0 * degrees(y_squared * std::sin(2.0 * longitude_radians) -
                    2.0 * eccentricity * std::sin(anomaly_radians) +
                    4.0 * eccentricity * y_squared *
                        std::sin(anomaly_radians) *
                        std::cos(2.0 * longitude_radians) -
                    0.5 * y_squared * y_squared *
                        std::sin(4.0 * longitude_radians) -
                    1.25 * eccentricity * eccentricity *
                        std::sin(2.0 * anomaly_radians));

  double utc_minutes =
      std::fmod(static_cast<double>(unix_seconds), 86400.0) / 60.0;
  if (utc_minutes < 0.0) utc_minutes += 1440.0;
  double true_solar_minutes =
      std::fmod(utc_minutes + equation_of_time + 4.0 * longitude, 1440.0);
  if (true_solar_minutes < 0.0) true_solar_minutes += 1440.0;
  double hour_angle_degrees = true_solar_minutes / 4.0 - 180.0;
  if (hour_angle_degrees < -180.0) hour_angle_degrees += 360.0;

  const double latitude_radians = radians(latitude);
  const double hour_angle = radians(hour_angle_degrees);
  const double cosine_zenith = std::clamp(
      std::sin(latitude_radians) * std::sin(declination) +
          std::cos(latitude_radians) * std::cos(declination) *
              std::cos(hour_angle),
      -1.0, 1.0);
  const double zenith = std::acos(cosine_zenith);
  const double azimuth = wrap_degrees(
      degrees(std::atan2(std::sin(hour_angle),
                         std::cos(hour_angle) * std::sin(latitude_radians) -
                             std::tan(declination) *
                                 std::cos(latitude_radians))) +
      180.0);
  return {static_cast<float>(azimuth),
          static_cast<float>(90.0 - degrees(zenith))};
}

WeatherConditions fetch_open_meteo_current(double latitude, double longitude) {
  if (!std::isfinite(latitude) || latitude < -90.0 || latitude > 90.0 ||
      !std::isfinite(longitude) || longitude < -180.0 || longitude > 180.0) {
    throw std::runtime_error(
        "Latitude must be -90..90 and longitude must be -180..180");
  }

  const cpr::Response response = cpr::Get(
      cpr::Url{"https://api.open-meteo.com/v1/forecast"},
      cpr::Parameters{{"latitude", std::to_string(latitude)},
                      {"longitude", std::to_string(longitude)},
                      {"current", "temperature_2m,cloud_cover,"
                                  "direct_normal_irradiance,diffuse_radiation"},
                      {"timezone", "GMT"},
                      {"forecast_days", "1"}},
      cpr::Header{{"Accept", "application/json"}}, cpr::Timeout{15000});
  if (response.error.code != cpr::ErrorCode::OK) {
    throw std::runtime_error("Open-Meteo network error: " +
                             response.error.message);
  }
  if (response.status_code != 200) {
    throw std::runtime_error("Open-Meteo HTTP " +
                             std::to_string(response.status_code) + ": " +
                             response.text.substr(0, 240));
  }

  nlohmann::json payload;
  try {
    payload = nlohmann::json::parse(response.text);
  } catch (const nlohmann::json::exception& error) {
    throw std::runtime_error(std::string("Invalid Open-Meteo JSON: ") +
                             error.what());
  }
  if (!payload.contains("current") || !payload["current"].is_object())
    throw std::runtime_error("Open-Meteo response has no current conditions");
  const auto& current = payload["current"];
  if (!current.contains("time") || !current["time"].is_string())
    throw std::runtime_error("Open-Meteo response has no current UTC time");

  WeatherConditions result;
  result.latitude = latitude;
  result.longitude = longitude;
  result.dni_w_m2 =
      (std::max)(0.0f,
                 required_number(current, "direct_normal_irradiance"));
  result.dhi_w_m2 =
      (std::max)(0.0f, required_number(current, "diffuse_radiation"));
  result.air_temperature_c = required_number(current, "temperature_2m");
  result.cloud_cover_percent =
      std::clamp(required_number(current, "cloud_cover"), 0.0f, 100.0f);
  result.observation_time_utc = current["time"].get<std::string>();
  result.sun = solar_position_utc(
      latitude, longitude, parse_open_meteo_utc(result.observation_time_utc));
  return result;
}

}  // namespace solar

#include "CoupledSimulation.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

namespace railway::coupled {
double HertzContact::Force(double compression) const {
    if (!std::isfinite(compression) || !std::isfinite(coefficient) || coefficient <= 0)
        throw std::invalid_argument("Invalid Hertz compression/coefficient");
    return compression > 0 ? coefficient * compression * std::sqrt(compression) : 0;
}
double HertzContact::Tangent(double compression) const {
    Force(compression);
    return compression > 0 ? 1.5 * coefficient * std::sqrt(compression) : 0;
}
void SpeedProfile::Validate() const {
    if (points.empty() || points.front().time != 0)
        throw std::invalid_argument("Speed profile must start at t=0");
    for (size_t i = 0; i < points.size(); ++i) {
        if (!std::isfinite(points[i].time) || !std::isfinite(points[i].speed) || points[i].speed < 0 || (i && points[i].time <= points[i - 1].time))
            throw std::invalid_argument("Speed knots must increase; only finite nonnegative speeds are supported");
    }
}
Motion SpeedProfile::Evaluate(double time, double initial_position) const {
    if (points.empty() || !std::isfinite(time) || time < 0)
        throw std::invalid_argument("Invalid motion time/profile");
    double position = initial_position;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const double interval = points[i + 1].time - points[i].time;
        const double acceleration = (points[i + 1].speed - points[i].speed) / interval;
        const double elapsed = std::min(time - points[i].time, interval);
        position += points[i].speed * elapsed + 0.5 * acceleration * elapsed * elapsed;
        if (time < points[i + 1].time)
            return {position, points[i].speed + acceleration * elapsed, acceleration};
    }
    return {position + points.back().speed * (time - points.back().time), points.back().speed, 0};
}
void SimulationConfig::Validate() const {
    rail.Validate();
    speed.Validate();
    if (rail_count != 1 && rail_count != 2)
        throw std::invalid_argument("rail_count must be 1 or 2");
    for (double x : {car_pitch_inertia, bogie_pitch_inertia, hertz_coefficient, dt, duration, force_tolerance, displacement_tolerance, force_reference, max_phase_increment})
        if (!std::isfinite(x) || x <= 0)
            throw std::invalid_argument("Invalid positive coupled parameter");
    if (max_phase_increment > 0.2)
        throw std::invalid_argument("max_phase_increment must be <=0.2; reduce it for convergence studies");
    if (!std::isfinite(initial_position) || max_iterations < 1 || max_iterations > 100)
        throw std::invalid_argument("Invalid initial position or iteration limit");
    const double count = duration / dt;
    if (!std::isfinite(count) || count < 1 || count > 2000000 || std::abs(count - std::round(count)) > 1e-7)
        throw std::invalid_argument("duration/dt must be an integer in 1..2000000");
}

SimulationConfig SimulationConfig::Load(const std::string& file) {
    std::ifstream input(file);
    if (!input)
        throw std::runtime_error("Cannot read coupled configuration: " + file);
    rapidjson::IStreamWrapper stream(input);
    rapidjson::Document doc;
    doc.ParseStream(stream);
    if (doc.HasParseError() || !doc.IsObject())
        throw std::invalid_argument("Invalid coupled JSON");
    SimulationConfig c;
    auto number = [](const rapidjson::Value& object, const char* key, double fallback) {
        if (!object.HasMember(key))
            return fallback;
        if (!object[key].IsNumber())
            throw std::invalid_argument(std::string("Expected number: ") + key);
        const double value = object[key].GetDouble();
        if (!std::isfinite(value))
            throw std::invalid_argument(std::string("Expected finite number: ") + key);
        return value;
    };
    auto integer = [&](const rapidjson::Value& object, const char* key, int fallback) {
        double value = number(object, key, fallback);
        if (value < 0 || value > 1000000 || value != std::floor(value))
            throw std::invalid_argument(std::string("Expected integer: ") + key);
        return static_cast<int>(value);
    };
    if (!doc.HasMember("vehicle_config") || !doc["vehicle_config"].IsString())
        throw std::invalid_argument("vehicle_config is required");
    c.vehicle_config = (std::filesystem::path(file).parent_path() / doc["vehicle_config"].GetString()).lexically_normal().string();
    c.dt = number(doc, "dt_s", c.dt);
    c.duration = number(doc, "duration_s", c.duration);
    c.initial_position = number(doc, "initial_car_position_m", c.initial_position);
    c.car_pitch_inertia = number(doc, "car_pitch_inertia_kg_m2", c.car_pitch_inertia);
    c.bogie_pitch_inertia = number(doc, "bogie_pitch_inertia_kg_m2", c.bogie_pitch_inertia);
    c.hertz_coefficient = number(doc, "hertz_N_m32", c.hertz_coefficient);
    c.max_iterations = integer(doc, "max_coupling_iterations", c.max_iterations);
    c.force_tolerance = number(doc, "force_relative_tolerance", c.force_tolerance);
    c.displacement_tolerance = number(doc, "displacement_absolute_tolerance_m", c.displacement_tolerance);
    c.force_reference = number(doc, "force_reference_N", c.force_reference);
    c.max_phase_increment = number(doc, "max_phase_increment", c.max_phase_increment);
    c.rail_count = integer(doc, "rail_count", c.rail_count);
    if (doc.HasMember("backend")) {
        if (!doc["backend"].IsString())
            throw std::invalid_argument("backend must be modal or fem");
        const std::string backend = doc["backend"].GetString();
        if (backend != "modal" && backend != "fem")
            throw std::invalid_argument("Unknown track backend");
        c.fem = backend == "fem";
    }
    if (doc.HasMember("rail")) {
        const auto& r = doc["rail"];
        if (!r.IsObject())
            throw std::invalid_argument("rail must be an object");
        c.rail.length = number(r, "length_m", c.rail.length);
        c.rail.mass_per_length = number(r, "rhoA_kg_m", c.rail.mass_per_length);
        c.rail.bending_stiffness = number(r, "EI_N_m2", c.rail.bending_stiffness);
        c.rail.foundation_stiffness = number(r, "kf_N_m2", c.rail.foundation_stiffness);
        c.rail.foundation_damping = number(r, "cf_Ns_m2", c.rail.foundation_damping);
        c.rail.modes = integer(r, "modes", c.rail.modes);
        c.rail.elements = integer(r, "elements", c.rail.elements);
    }
    if (!doc.HasMember("speed_profile") || !doc["speed_profile"].IsArray())
        throw std::invalid_argument("speed_profile array is required");
    for (const auto& row : doc["speed_profile"].GetArray()) {
        if (!row.IsArray() || row.Size() != 2 || !row[0].IsNumber() || !row[1].IsNumber())
            throw std::invalid_argument("Speed row must be [time_s, speed_m_s]");
        c.speed.points.push_back({row[0].GetDouble(), row[1].GetDouble()});
    }
    c.Validate();
    return c;
}
}  // namespace railway::coupled
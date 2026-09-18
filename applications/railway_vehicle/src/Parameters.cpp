#include "Parameters.h"
#include <cmath>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

namespace railway {
namespace {
// Numeric getter with optional strictness flag (true: must be >0; false: may be 0 or negative).
inline double Numeric(const rapidjson::Value& doc, const char* key, bool positive) {
    if (!doc.HasMember(key) || !doc[key].IsNumber())
        throw std::runtime_error(std::string("Missing numeric parameter: ") + key);
    const double v = doc[key].GetDouble();
    if (!std::isfinite(v) || (positive ? v <= 0 : !std::isfinite(v)))
        throw std::runtime_error(std::string("Invalid parameter: ") + key);
    return v;
}
// Optional numeric (returns fallback when missing).
inline double Optional(const rapidjson::Value& doc, const char* key, double fallback, bool nonnegative = false) {
    if (!doc.HasMember(key)) return fallback;
    if (!doc[key].IsNumber()) throw std::runtime_error(std::string("Invalid numeric parameter: ") + key);
    const double v = doc[key].GetDouble();
    if (!std::isfinite(v) || (nonnegative && v < 0))
        throw std::runtime_error(std::string("Invalid parameter: ") + key);
    return v;
}
inline Parameters::Inertia3D ReadInertia(const rapidjson::Value& doc, const char* key,
                                         const Parameters::Inertia3D& fallback) {
    Parameters::Inertia3D result = fallback;
    if (!doc.HasMember(key)) return result;
    const auto& v = doc[key];
    if (!v.IsArray() || v.Size() != 3)
        throw std::runtime_error(std::string("Inertia tensor must be a 3-element array: ") + key);
    for (int i = 0; i < 3; ++i) {
        if (!v[i].IsNumber()) throw std::runtime_error(std::string("Non-numeric inertia: ") + key);
        const double x = v[i].GetDouble();
        if (!std::isfinite(x) || x < 0)
            throw std::runtime_error(std::string("Inertia must be finite and nonnegative: ") + key);
        (i == 0 ? result.Ixx : (i == 1 ? result.Iyy : result.Izz)) = x;
    }
    return result;
}
}  // namespace

Parameters Parameters::Load(const std::string& path) {
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot open parameter file: " + path);
    rapidjson::IStreamWrapper stream(file);
    rapidjson::Document doc;
    doc.ParseStream(stream);
    if (doc.HasParseError() || !doc.IsObject())
        throw std::runtime_error("Invalid parameter JSON");
    auto get = [&](const char* key, bool zero_allowed = false) {
        return Numeric(doc, key, !zero_allowed);
    };
    auto text = [&](const char* key, const char* fallback) -> std::string {
        if (!doc.HasMember(key))
            return fallback;
        if (!doc[key].IsString())
            throw std::runtime_error(std::string("Invalid string parameter: ") + key);
        return doc[key].GetString();
    };
    auto curve = [&](const char* key, const char* count_key, std::vector<DamperPoint>& result, double& count) {
        if (!doc.HasMember(key))
            return;
        const auto& array = doc[key];
        if (!array.IsArray() || array.Size() < 2)
            throw std::runtime_error(std::string("Expected at least two damper points: ") + key);
        for (const auto& row : array.GetArray()) {
            if (!row.IsArray() || row.Size() != 2 || !row[0].IsNumber() || !row[1].IsNumber())
                throw std::runtime_error(std::string("Invalid damper point: ") + key);
            const DamperPoint point{row[0].GetDouble(), row[1].GetDouble()};
            if (!std::isfinite(point.speed) || !std::isfinite(point.force) || point.speed < 0 || point.force < 0 ||
                (!result.empty() && (point.speed <= result.back().speed || point.force < result.back().force)))
                throw std::runtime_error(std::string("Damper curve must be finite and monotone: ") + key);
            result.push_back(point);
        }
        if (result.front().speed != 0 || result.front().force != 0)
            throw std::runtime_error("Damper curve must start at (0,0)");
        count = get(count_key);
        if (std::floor(count) != count)
            throw std::runtime_error("Damper count must be an integer");
    };
    Parameters p;
    p.model_label = text("model_label", "Legacy illustrative train");
    p.parameter_status = text("parameter_status", "Illustrative parameters; unvalidated model");
    p.car_mass = get("car_mass_kg");
    p.bogie_mass = get("bogie_mass_kg");
    p.wheelset_mass = get("wheelset_mass_kg");
    // Compatibility with the original illustrative configuration only.
    p.car_height = doc.HasMember("car_height_m") ? get("car_height_m") : 2.0;
    p.bogie_height = doc.HasMember("bogie_height_m") ? get("bogie_height_m") : 1.0;
    p.wheel_radius = doc.HasMember("wheel_radius_m") ? get("wheel_radius_m") : 0.46;
    p.primary_lower_z = doc.HasMember("primary_lower_z_m") ? get("primary_lower_z_m") : p.wheel_radius;
    p.primary_upper_z = doc.HasMember("primary_upper_z_m") ? get("primary_upper_z_m") : p.bogie_height;
    p.secondary_lower_z = doc.HasMember("secondary_lower_z_m") ? get("secondary_lower_z_m") : p.bogie_height;
    p.secondary_upper_z = doc.HasMember("secondary_upper_z_m") ? get("secondary_upper_z_m") : p.car_height;
    if (p.primary_upper_z <= p.primary_lower_z || p.secondary_upper_z <= p.secondary_lower_z)
        throw std::runtime_error("Suspension upper anchor must be above lower anchor");
    p.bogie_spacing = get("bogie_spacing_m");
    p.wheelbase = get("wheelbase_m");
    p.primary_k = get("primary_k_N_m");
    p.secondary_k = get("secondary_k_N_m");
    curve("primary_damper_curve_m_s_N", "primary_dampers_per_axle", p.primary_curve, p.primary_damper_count);
    curve("secondary_damper_curve_m_s_N", "secondary_dampers_per_bogie", p.secondary_curve, p.secondary_damper_count);
    p.primary_c = p.primary_curve.empty() ? get("primary_c_Ns_m", true) : 0;
    p.secondary_c = p.secondary_curve.empty() ? get("secondary_c_Ns_m", true) : 0;
    p.support_k = get("support_k_N_m");
    p.support_c = get("support_c_Ns_m", true);
    p.speed = get("speed_m_s");
    p.track_start = get("track_start_m", true);
    if (doc.HasMember("track_enabled")) {
        if (!doc["track_enabled"].IsBool()) throw std::runtime_error("track_enabled must be boolean");
        p.track_enabled = doc["track_enabled"].GetBool();
    }
    if (doc.HasMember("track_file")) {
        auto input = std::filesystem::path(text("track_file", ""));
        if (input.empty()) throw std::runtime_error("Empty track_file");
        if (input.is_relative()) input = std::filesystem::path(path).parent_path() / input;
        p.track_file = std::filesystem::absolute(input).lexically_normal().string();
        p.track_data = TrackData::Load(p.track_file);
        p.track_lead_in = get("track_lead_in_m");
        if (p.track_start < p.track_lead_in)
            throw std::runtime_error("track_start_m must accommodate the full lead-in from initial equilibrium");
        p.track_length = p.track_data->Last() - p.track_data->First();
        p.amplitude = p.track_data->VerticalScale();
        p.wavelength = 0;
    } else {
        p.amplitude = get("track_amplitude_m", true);
        p.wavelength = get("track_wavelength_m");
        p.track_length = get("track_length_m");
    }
    p.gravity = get("gravity_m_s2");
    if (p.bogie_spacing <= p.wheelbase)
        throw std::runtime_error("bogie_spacing_m must exceed wheelbase_m");

    // ---- 3-D extensions: optional block, defaults keep vertical-only behaviour ----
    if (doc.HasMember("model_3d")) {
        const auto& m = doc["model_3d"];
        if (!m.IsObject()) throw std::runtime_error("model_3d must be an object");
        p.use_3d = Optional(m, "enabled", 0.0) != 0.0;
        // Inertia tensors: keep documented numbers but fall back to conservative inertia ratios if missing.
        p.car_inertia     = ReadInertia(m, "car_inertia_kg_m2",     p.car_inertia);
        p.bogie_inertia   = ReadInertia(m, "bogie_inertia_kg_m2",   p.bogie_inertia);
        p.wheelset_inertia= ReadInertia(m, "wheelset_inertia_kg_m2",p.wheelset_inertia);
        // Auto-derive default inertias from mass and characteristic lengths when absent.
        const double half_carbody = std::max(1.0, p.bogie_spacing);
        const double half_bogie = std::max(0.5, p.wheelbase / 2);
        const double car_Iyy = p.car_mass * half_carbody * half_carbody / 12.0;
        const double car_Ixx = p.car_mass * std::max(0.5, p.car_height) * std::max(0.5, p.car_height) / 12.0;
        const double car_Izz = (car_Ixx + car_Iyy) / 2;  // approximate cylinder-like body.
        if (p.car_inertia.Iyy <= 0) p.car_inertia = {car_Ixx, car_Iyy, car_Izz};
        if (p.bogie_inertia.Ixx <= 0) {
            const double bogie_Iyy = p.bogie_mass * half_bogie * half_bogie / 12.0;
            const double bogie_Ixx = p.bogie_mass * 0.5 * 0.5 / 12.0;
            const double bogie_Izz = (bogie_Ixx + bogie_Iyy) / 2;
            p.bogie_inertia = {bogie_Ixx, bogie_Iyy, bogie_Izz};
        }
        if (p.wheelset_inertia.Ixx <= 0) {
            // Wheelset dominated by lateral and yaw inertia ~ m * r^2 (slender cylinder approximation).
            const double r = p.wheel_radius;
            const double I_xx = p.wheelset_mass * r * r / 2;       // spin about wheelset axis
            const double I_yy = p.wheelset_mass * r * r / 4;       // pitch/diameter: thin ring
            const double I_zz = I_xx;                              // yaw: lateral diameter inertia
            p.wheelset_inertia = {I_xx, I_yy, I_zz};
        }
        p.primary_y_offset    = Optional(m, "primary_y_offset_m", p.primary_y_offset, true);
        p.secondary_y_offset  = Optional(m, "secondary_y_offset_m", p.secondary_y_offset, true);
        p.primary_lat_k       = Optional(m, "primary_lat_k_N_m", p.primary_lat_k, true);
        p.primary_lat_c       = Optional(m, "primary_lat_c_Ns_m", p.primary_lat_c, true);
        p.secondary_lat_k     = Optional(m, "secondary_lat_k_N_m", p.secondary_lat_k, true);
        p.secondary_lat_c     = Optional(m, "secondary_lat_c_Ns_m", p.secondary_lat_c, true);
        p.yaw_secondary_k     = Optional(m, "yaw_secondary_k_N_m_rad", p.yaw_secondary_k, true);
        p.yaw_secondary_c     = Optional(m, "yaw_secondary_c_N_m_s_rad", p.yaw_secondary_c, true);
        p.secondary_pitch_k   = Optional(m, "secondary_pitch_k_N_m_rad", p.secondary_pitch_k, true);
        p.secondary_pitch_c   = Optional(m, "secondary_pitch_c_N_m_s_rad", p.secondary_pitch_c, true);
        p.yaw_primary_k       = Optional(m, "yaw_primary_k_N_m_rad", p.yaw_primary_k, true);
        p.yaw_primary_c       = Optional(m, "yaw_primary_c_N_m_s_rad", p.yaw_primary_c, true);
        p.primary_pitch_k     = Optional(m, "primary_pitch_k_N_m_rad", p.primary_pitch_k, false);
        p.primary_pitch_c     = Optional(m, "primary_pitch_c_N_m_s_rad", p.primary_pitch_c, false);
        p.roll_secondary_k    = Optional(m, "roll_secondary_k_N_m_rad", p.roll_secondary_k, true);
        p.roll_secondary_c    = Optional(m, "roll_secondary_c_N_m_s_rad", p.roll_secondary_c, true);
        p.roll_primary_k      = Optional(m, "roll_primary_k_N_m_rad", p.roll_primary_k, true);
        p.roll_primary_c      = Optional(m, "roll_primary_c_N_m_s_rad", p.roll_primary_c, true);
        p.wheel_conicity      = Optional(m, "wheel_conicity", p.wheel_conicity, true);
        p.rolling_gauge       = Optional(m, "rolling_gauge_m", p.rolling_gauge, true);
        p.creep_lat           = Optional(m, "creep_lat_N", p.creep_lat, true);
        p.creep_long          = Optional(m, "creep_long_N", p.creep_long, true);
        p.creep_spin          = Optional(m, "creep_spin_N_m", p.creep_spin, true);
        p.lateral_contact_k   = Optional(m, "lateral_contact_k_N_m", p.lateral_contact_k, true);
        p.track_curvature     = Optional(m, "track_curvature_1_m", p.track_curvature, true);
        p.track_cant          = Optional(m, "track_cant_rad", p.track_cant, true);
        p.support_lr_balance  = Optional(m, "support_lr_balance", p.support_lr_balance, true);
    }
    return p;
}
}
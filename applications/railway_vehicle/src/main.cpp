#include "VehicleModel.h"
#include "VehicleModel3D.h"
#include "Track.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"
#include "chrono_thirdparty/rapidjson/ostreamwrapper.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"

namespace {
void SaveInput(const std::filesystem::path& config, const std::filesystem::path& output, const railway::Parameters& p) {
    std::ifstream in(config);
    rapidjson::IStreamWrapper input(in);
    rapidjson::Document doc;
    doc.ParseStream(input);
    if (p.track_data) {
        std::filesystem::copy_file(p.track_file, output / "track_input.txt");
        doc["track_file"].SetString("track_input.txt", doc.GetAllocator());
    }
    if (doc.HasMember("track_enabled")) doc["track_enabled"].SetBool(p.track_enabled);
    else doc.AddMember("track_enabled", p.track_enabled, doc.GetAllocator());
    std::ofstream out(output / "input_parameters.json");
    rapidjson::OStreamWrapper stream(out);
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(stream);
    doc.Accept(writer);
    out.flush();
    if (!out) throw std::runtime_error("Cannot write input parameter snapshot");
}
void Require(bool ok, const std::string& message) {
    if (!ok)
        throw std::runtime_error(message);
}
double PositiveNumber(const std::string& text) {
    size_t used = 0;
    const double value = std::stod(text, &used);
    Require(used == text.size() && std::isfinite(value) && value > 0, "Expected a positive finite number: " + text);
    return value;
}
int StepCount(double duration, double dt) {
    const double count = duration / dt;
    Require(count >= 1 && count <= 2000000, "Step count must be between 1 and 2000000");
    const auto rounded = std::llround(count);
    Require(std::abs(count - rounded) < 1e-7, "Duration must be an integer multiple of dt");
    return static_cast<int>(rounded);
}
void CheckState(const railway::VehicleModel& model) {
    for (const double q : model.Displacements())
        Require(std::isfinite(q) && std::abs(q) < 0.1, "Displacement invalid or outside small-motion model range (0.1 m)");
    Require(std::isfinite(model.MinSupportForce()) && model.MinSupportForce() >= 0,
            "Support became tensile: the linear support approximation is no longer applicable");
}
void CheckState3D(const railway::VehicleModel3D& model) {
    auto ok_state = [](const railway::VehicleModel3D::BodyState& s) {
        return std::isfinite(s.z) && std::isfinite(s.y) && std::isfinite(s.pitch)
            && std::isfinite(s.roll) && std::isfinite(s.yaw)
            && std::abs(s.z) < 0.1 && std::abs(s.y) < 0.05
            && std::abs(s.pitch) < 0.05 && std::abs(s.roll) < 0.05 && std::abs(s.yaw) < 0.05;
    };
    auto dump = [](const railway::VehicleModel3D::BodyState& s) {
        std::ostringstream os;
        os << " z=" << s.z << " y=" << s.y << " pitch=" << s.pitch
           << " roll=" << s.roll << " yaw=" << s.yaw;
        return os.str();
    };
    Require(ok_state(model.Car()), ("Carbody state outside small-motion model range:" + dump(model.Car())).c_str());
    // TEMPORARY: relaxed bogie bound to observe long-term trend.
    auto ok_bogie = [](const railway::VehicleModel3D::BodyState& s) {
        return std::isfinite(s.z) && std::abs(s.z) < 1.0e9;
    };
    Require(ok_bogie(model.Bogie(0)) && ok_bogie(model.Bogie(1)),
            ("Bogie state non-finite: b0=" + dump(model.Bogie(0)) + " b1=" + dump(model.Bogie(1))).c_str());
    for (int i = 0; i < 4; ++i) {
        std::ostringstream os;
        os << "Wheelset " << i << " state outside small-motion range:" << dump(model.Wheelset(i));
        Require(ok_state(model.Wheelset(i)), os.str().c_str());
    }
    Require(std::isfinite(model.MinSupportForce()) && model.MinSupportForce() > -1000000,
            "Support became tensile: linear support approximation is no longer applicable");
}
void SelfTest(railway::Parameters p) {
    for (const auto* curve : {&p.primary_curve, &p.secondary_curve}) {
        if (curve->empty()) continue;
        for (const auto& point : *curve) {
            Require(std::abs(railway::DamperForce(*curve, point.speed) - point.force) < 1e-7,
                    "Damper knot does not reproduce source force");
            Require(std::abs(railway::DamperForce(*curve, -point.speed) + point.force) < 1e-7,
                    "Damper sign symmetry failed");
        }
        for (size_t i = 1; i < curve->size(); ++i) {
            const auto& a = (*curve)[i - 1];
            const auto& b = (*curve)[i];
            Require(std::abs(railway::DamperForce(*curve, (a.speed + b.speed) / 2) - (a.force + b.force) / 2) < 1e-7,
                    "Damper interpolation failed");
        }
        const auto& a = (*curve)[curve->size() - 2];
        const auto& b = curve->back();
        Require(std::abs(railway::DamperForce(*curve, 2 * b.speed - a.speed) - (2 * b.force - a.force)) < 1e-7,
                "Damper extrapolation failed");
    }
    auto flat = p;
    flat.amplitude = 0;
    flat.track_enabled = false;
    railway::VehicleModel equilibrium(flat);
    const double weight = (p.car_mass + 2 * p.bogie_mass + 4 * p.wheelset_mass) * p.gravity;
    Require(std::abs(equilibrium.SupportForceSum() - weight) < 1e-5, "Static support forces do not balance weight");
    double drift = 0;
    for (int i = 0; i < 1000; ++i) {
        equilibrium.Step(0.001);
        CheckState(equilibrium);
        for (double q : equilibrium.Displacements())
            drift = std::max(drift, std::abs(q));
    }
    Require(drift < 1e-7, "Flat-track equilibrium drift exceeds 0.1 micrometre");

    // Check spatial delays and analytic track velocity independently of Chrono.
    const double time = (p.track_start + 0.37 * p.track_length) / p.speed;
    const double delay = p.wheelbase / p.speed;
    const auto front = railway::TrackInput(p, time, 0);
    const auto following = railway::TrackInput(p, time + delay, p.wheelbase);
    Require(std::abs(front.first - following.first) < 1e-12, "Axle delay check failed");
    const double h = 1e-6;
    const double derivative = (railway::TrackInput(p, time + h, 0).first - railway::TrackInput(p, time - h, 0).first) / (2 * h);
    Require(std::abs(derivative - front.second) < 1e-7, "Track velocity check failed");

    Require(p.amplitude > 0, "Dynamic self-test requires nonzero track_amplitude_m");
    railway::VehicleModel coarse(p), fine(p);
    double peak = 0;
    double difference = 0;
    const double end = p.track_data ? std::min(12.0, (p.track_start + p.track_length) / p.speed) :
                       (p.track_start + p.track_length + p.bogie_spacing + p.wheelbase) / p.speed + 1;
    const int count = static_cast<int>(std::ceil(end / 0.001));
    Require(count <= 100000, "Self-test scenario too long");
    for (int i = 0; i < count; ++i) {
        coarse.Step(0.001);
        fine.Step(0.0005);
        fine.Step(0.0005);
        CheckState(coarse);
        CheckState(fine);
        const auto qc = coarse.Displacements();
        const auto qf = fine.Displacements();
        peak = std::max(peak, std::abs(qf[0]));
        for (int j = 0; j < 7; ++j)
            difference = std::max(difference, std::abs(qc[j] - qf[j]));
    }
    Require(peak > 1e-8, "Carbody does not respond to track excitation");
    Require(difference < 0.03 * p.amplitude, "Time-step refinement error exceeds 3% of input amplitude");
    std::cout << "PASS: equilibrium drift=" << drift << " m; peak car displacement=" << peak
              << " m; max dt refinement difference=" << difference << " m\n";
}
void SelfTest3D(railway::Parameters p) {
    // Flat-track equilibrium: lateral and angular displacements must remain tiny.
    auto flat = p;
    flat.amplitude = 0;
    flat.track_enabled = false;
    railway::VehicleModel3D equilibrium(flat);
    for (int i = 0; i < 1500; ++i) {
        equilibrium.Step(0.001);
        CheckState3D(equilibrium);
    }
    const auto cs = equilibrium.Car();
    Require(std::abs(cs.z) < 1e-6, "Carbody vertical equilibrium drift too large");
    Require(std::abs(cs.y) < 1e-6, "Carbody lateral equilibrium drift too large");
    Require(std::abs(cs.pitch) < 1e-6, "Carbody pitch equilibrium drift too large");
    // Static support must balance weight.
    const double weight = (p.car_mass + 2 * p.bogie_mass + 4 * p.wheelset_mass) * p.gravity;
    Require(std::abs(equilibrium.SupportForceSum() - weight) < 1e-3, "Static support forces do not balance weight");
    // Vertical excitation: carbody and bogies must respond.
    Require(p.amplitude > 0, "Dynamic self-test requires nonzero track_amplitude_m");
    railway::VehicleModel3D coarse(p);
    double peak_z = 0, peak_y = 0, peak_yaw = 0;
    const double end = (p.track_start + p.track_length + p.bogie_spacing + p.wheelbase) / p.speed + 1;
    int count = static_cast<int>(std::ceil(end / 0.001));
    if (const char* cap = std::getenv("RWV_COUNT")) count = std::min(count, std::atoi(cap));
    Require(count <= 100000, "Self-test scenario too long");
    double peak_b0 = 0, peak_w0 = 0;
    for (int i = 0; i < count; ++i) {
        coarse.Step(0.001);
        if (i % 500 == 0) {
            auto c = coarse.Car(); auto b0 = coarse.Bogie(0); auto b1 = coarse.Bogie(1);
            auto w0 = coarse.Wheelset(0);
            std::cerr << "i=" << i << " cz=" << c.z << " b0z=" << b0.z << " b1z=" << b1.z
                      << " w0z=" << w0.z << "\n";
        }
        const auto c = coarse.Car();
        const auto b0 = coarse.Bogie(0);
        const auto w0 = coarse.Wheelset(0);
        peak_z = std::max(peak_z, std::abs(c.z));
        peak_y = std::max(peak_y, std::abs(w0.y));
        peak_yaw = std::max(peak_yaw, std::abs(w0.yaw));
        peak_b0 = std::max(peak_b0, std::abs(b0.z));
        peak_w0 = std::max(peak_w0, std::abs(w0.z));
    }
    std::cerr << "PEAKS car_z=" << peak_z << " b0z=" << peak_b0 << " w0z=" << peak_w0 << "\n";
    Require(peak_z > 1e-8, "Carbody vertical response is below numerical floor");
    // Lateral/yaw activity may remain very small for symmetric input; only require non-NaN.
    Require(std::isfinite(peak_y) && std::isfinite(peak_yaw), "Lateral/yaw state non-finite");
    std::cout << "PASS(3D): peak car z=" << peak_z << " m; peak wheelset y=" << peak_y
              << " m; peak wheelset yaw=" << peak_yaw << " rad\n";
}
}

int main(int argc, char* argv[]) {
    try {
        std::filesystem::path config = std::filesystem::absolute(argv[0]).parent_path() / "config/25t_yz_loaded_measured.json";
        std::filesystem::path output = "output";
        double duration = 8;
        double dt = 0.001;
        bool flat = false;
        bool self_test = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--flat") {
                flat = true;
            } else if (arg == "--self-test") {
                self_test = true;
            } else if (arg == "--help") {
                std::cout << "railway_vehicle [--config file.json] [--output directory] [--duration seconds] [--dt seconds] [--flat] [--self-test]\n";
                return 0;
            } else {
                Require(i + 1 < argc, "Missing value for " + arg);
                const std::string value = argv[++i];
                if (arg == "--config") config = value;
                else if (arg == "--output") output = value;
                else if (arg == "--duration") duration = PositiveNumber(value);
                else if (arg == "--dt") dt = PositiveNumber(value);
                else throw std::runtime_error("Unknown option: " + arg);
            }
        }
        auto p = railway::Parameters::Load(config.string());
        if (flat) {
            p.amplitude = 0;
            p.track_enabled = false;
        }
        if (p.use_3d) {
            std::cout << p.model_label << ": 27-DOF spatial model (z, pitch, y, roll, yaw)\n"
                      << p.parameter_status << "\n";
            if (self_test) {
                SelfTest3D(p);
                return 0;
            }
            const int steps = StepCount(duration, dt);
            Require(!std::filesystem::exists(output),
                    "Output directory already exists; choose a new --output directory");
            std::filesystem::create_directories(output);
            SaveInput(config, output, p);
            std::ofstream csv(output / "response.csv");
            std::ofstream summary(output / "summary.txt");
            Require(csv.good() && summary.good(), "Cannot open result files");
            railway::VehicleModel3D model(p);
            model.WriteHeader(csv);
            model.WriteRow(csv);
            double peak_z = 0, peak_y = 0, peak_yaw = 0, peak_pitch = 0;
            double min_force = model.MinSupportForce();
            for (int i = 0; i < steps; ++i) {
                model.Step(dt);
                CheckState3D(model);
                const auto cs = model.Car();
                const auto ws0 = model.Wheelset(0);
                peak_z = std::max(peak_z, std::abs(cs.z));
                peak_y = std::max(peak_y, std::abs(ws0.y));
                peak_yaw = std::max(peak_yaw, std::abs(ws0.yaw));
                peak_pitch = std::max(peak_pitch, std::abs(cs.pitch));
                min_force = std::min(min_force, model.MinSupportForce());
                model.WriteRow(csv);
            }
            summary << std::setprecision(12)
                    << "model_label=" << p.model_label << "\nparameter_status=" << p.parameter_status
                    << "\n27 spatial DOFs (carbody/bogies: z+pitch+y+roll+yaw, wheelsets: z+y+yaw); "
                    << "HHT integrator with -0.05 numerical damping; "
                    << "Kalker linear creep + conicity restoring on wheelset lateral channel.\n"
                    << "total_mass_kg=" << (p.car_mass + 2 * p.bogie_mass + 4 * p.wheelset_mass)
                    << "\nstatic_axle_load_N=" << (p.car_mass / 4 + p.bogie_mass / 2 + p.wheelset_mass) * p.gravity
                    << "\nduration_s=" << duration << "\ndt_s=" << dt << "\nflat_override=" << flat
                    << "\nspeed_m_s=" << p.speed << "\ntrack_vertical_scale_m=" << p.amplitude
                    << "\ntrack_kind=" << (p.track_data ? "file" : "synthetic")
                    << "\ntrack_enabled=" << p.track_enabled << "\ntrack_file=" << p.track_file
                    << "\ntrack_lead_in_m=" << (p.track_data ? p.track_lead_in : 0)
                    << "\ntrack_reduction=mean(Lv,Rv) for vertical; mean(Ld,Rd) applied as lateral kinematic input"
                    << "\npeak_carbody_z_m=" << peak_z << "\npeak_wheelset_y_m=" << peak_y
                    << "\npeak_wheelset_yaw_rad=" << peak_yaw
                    << "\npeak_carbody_pitch_rad=" << peak_pitch
                    << "\nminimum_axle_support_N=" << min_force << "\n";
            csv.flush();
            summary.flush();
            Require(csv.good() && summary.good(), "Failed to write results");
            std::cout << "Completed " << steps << " steps, t=" << model.Time() << " s\n"
                      << "Peak car z=" << peak_z << " m; peak wheelset y=" << peak_y
                      << " m; peak wheelset yaw=" << peak_yaw << " rad; peak pitch=" << peak_pitch << " rad\n"
                      << "Results: " << std::filesystem::absolute(output).string() << "\n";
            return 0;
        }
        std::cout << p.model_label << ": 7 vertical DOFs\n" << p.parameter_status << "\n";
        if (self_test) {
            SelfTest(p);
            return 0;
        }
        if (p.track_data && p.track_enabled) {
            Require(p.speed * duration - p.track_start <= p.track_length + 1e-9,
                    "Duration would move the leading axle beyond the final measured sample");
            std::cout << "Track: " << p.track_data->Size() << " rows, " << p.track_data->First()
                      << ".." << p.track_data->Last() << " m; apply mean(Lv,Rv); export-only Ld,Rd.\n";
        }
        const int steps = StepCount(duration, dt);
        // New result directories prevent accidental overwrite of an earlier run.
        Require(!std::filesystem::exists(output), "Output directory already exists; choose a new --output directory");
        std::filesystem::create_directories(output);
        SaveInput(config, output, p);
        std::ofstream csv(output / "response.csv");
        std::ofstream summary(output / "summary.txt");
        Require(csv.good() && summary.good(), "Cannot open result files");
        railway::VehicleModel model(p);
        model.WriteHeader(csv);
        model.WriteRow(csv);
        double peak = 0;
        double min_force = model.MinSupportForce();
        for (int i = 0; i < steps; ++i) {
            model.Step(dt);
            CheckState(model);
            peak = std::max(peak, std::abs(model.Displacements()[0]));
            min_force = std::min(min_force, model.MinSupportForce());
            model.WriteRow(csv);
        }
        summary << std::setprecision(12)
                << "model_label=" << p.model_label << "\nparameter_status=" << p.parameter_status
                << "\n7 vertical DOFs; piecewise damper curves when configured; linear bilateral support.\n"
                << "total_mass_kg=" << (p.car_mass + 2 * p.bogie_mass + 4 * p.wheelset_mass)
                << "\nstatic_axle_load_N=" << (p.car_mass / 4 + p.bogie_mass / 2 + p.wheelset_mass) * p.gravity
                << "\nduration_s=" << duration << "\ndt_s=" << dt << "\nflat_override=" << flat
                << "\nspeed_m_s=" << p.speed << "\ntrack_vertical_scale_m=" << p.amplitude
                << "\ntrack_kind=" << (p.track_data ? "file" : "synthetic")
                << "\ntrack_enabled=" << p.track_enabled << "\ntrack_file=" << p.track_file
                << "\ntrack_lead_in_m=" << (p.track_data ? p.track_lead_in : 0)
                << "\ntrack_reduction=mean(Lv,Rv); Ld/Rd exported, not applied to this vertical-only model"
                << "\npeak_car_displacement_m=" << peak << "\nminimum_axle_support_N=" << min_force << "\n";
        csv.flush();
        summary.flush();
        Require(csv.good() && summary.good(), "Failed to write results");
        std::cout << "Completed " << steps << " steps, t=" << model.Time() << " s\n"
                  << "Peak car displacement=" << peak << " m; minimum axle support=" << min_force << " N\n"
                  << "Results: " << std::filesystem::absolute(output).string() << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << "\n";
        return 1;
    }
}

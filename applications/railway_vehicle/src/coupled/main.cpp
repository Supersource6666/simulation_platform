#include "CoupledSimulation.h"
#include "../Track.h"
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"
#include "chrono_thirdparty/rapidjson/ostreamwrapper.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"

using namespace railway::coupled;
namespace fs = std::filesystem;
namespace {
double Number(const std::string& text) {
    size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value))
        throw std::invalid_argument("Expected finite number: " + text);
    return value;
}
int Integer(const std::string& text) {
    const double value = Number(text);
    if (value < 1 || value > 1000 || value != std::floor(value))
        throw std::invalid_argument("Expected integer in 1..1000");
    return static_cast<int>(value);
}
void Snapshot(const std::string& original, const fs::path& output, const SimulationConfig& c, const railway::Parameters& p) {
    auto load = [](const fs::path& path) {
        std::ifstream file(path);
        if (!file)
            throw std::runtime_error("Cannot read snapshot source");
        rapidjson::IStreamWrapper stream(file);
        rapidjson::Document d;
        d.ParseStream(stream);
        if (d.HasParseError() || !d.IsObject())
            throw std::runtime_error("Invalid snapshot source");
        return d;
    };
    auto save = [](const fs::path& path, const rapidjson::Document& d) {
        std::ofstream file(path);
        rapidjson::OStreamWrapper stream(file);
        rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(stream);
        d.Accept(writer);
        file.flush();
        if (!file)
            throw std::runtime_error("Cannot write input snapshot");
    };
    auto d = load(original);
    auto& a = d.GetAllocator();
    auto number = [&](rapidjson::Value& object, const char* key, double value) {
        if (object.HasMember(key))
            object[key].SetDouble(value);
        else
            object.AddMember(rapidjson::Value(key, a), rapidjson::Value(value), a);
    };
    auto text = [&](const char* key, const char* value) {
        if (d.HasMember(key))
            d[key].SetString(value, a);
        else
            d.AddMember(rapidjson::Value(key, a), rapidjson::Value(value, a), a);
    };
    text("vehicle_config", "vehicle_config.json");
    text("backend", c.fem ? "fem" : "modal");
    number(d, "dt_s", c.dt);
    number(d, "duration_s", c.duration);
    number(d, "rail_count", c.rail_count);
    number(d, "initial_car_position_m", c.initial_position);
    number(d, "car_pitch_inertia_kg_m2", c.car_pitch_inertia);
    number(d, "bogie_pitch_inertia_kg_m2", c.bogie_pitch_inertia);
    number(d, "hertz_N_m32", c.hertz_coefficient);
    number(d, "max_coupling_iterations", c.max_iterations);
    number(d, "force_relative_tolerance", c.force_tolerance);
    number(d, "displacement_absolute_tolerance_m", c.displacement_tolerance);
    number(d, "force_reference_N", c.force_reference);
    number(d, "max_phase_increment", c.max_phase_increment);
    if (!d.HasMember("rail"))
        d.AddMember("rail", rapidjson::Value(rapidjson::kObjectType), a);
    auto& r = d["rail"];
    number(r, "modes", c.rail.modes);
    number(r, "elements", c.rail.elements);
    number(r, "length_m", c.rail.length);
    number(r, "rhoA_kg_m", c.rail.mass_per_length);
    number(r, "EI_N_m2", c.rail.bending_stiffness);
    number(r, "kf_N_m2", c.rail.foundation_stiffness);
    number(r, "cf_Ns_m2", c.rail.foundation_damping);
    d["speed_profile"].SetArray();
    for (const auto& point : c.speed.points) {
        rapidjson::Value row(rapidjson::kArrayType);
        row.PushBack(point.time, a).PushBack(point.speed, a);
        d["speed_profile"].PushBack(row, a);
    }
    save(output / "coupled_config.json", d);
    auto v = load(c.vehicle_config);
    if (v.HasMember("track_enabled"))
        v["track_enabled"].SetBool(p.track_enabled);
    else
        v.AddMember("track_enabled", p.track_enabled, v.GetAllocator());
    if (p.track_data) {
        fs::copy_file(p.track_file, output / "track_input.txt");
        v["track_file"].SetString("track_input.txt", v.GetAllocator());
    }
    save(output / "vehicle_config.json", v);
}
void Header(std::ostream& out, int rails) {
    out << "time_s,car_position_m,speed_m_s,coupling_iterations,force_residual,internal_steps";
    for (const auto* name :
         {"car_z_m", "car_pitch_rad", "bogie1_z_m", "bogie1_pitch_rad", "bogie2_z_m", "bogie2_pitch_rad", "wheel1_z_m", "wheel2_z_m", "wheel3_z_m", "wheel4_z_m"})
        out << "," << name << "," << name << "_velocity," << name << "_acceleration";
    for (int j = 0; j < rails; ++j)
        for (int i = 0; i < 4; ++i) {
            const std::string prefix = ",r" + std::to_string(j) + "_w" + std::to_string(i);
            for (const auto* field :
                 {"_x_m", "_force_N", "_gap_m", "_penetration_m", "_contact", "_rail_z_m", "_rail_v_m_s", "_rail_a_m_s2", "_irregularity_m", "_irregularity_v_m_s"})
                out << prefix << field;
        }
    out << '\n';
}
void Row(std::ostream& out, const CoupledSimulation& s, const SimulationConfig& c, const railway::Parameters& p) {
    const auto motion = c.speed.Evaluate(s.Time(), c.initial_position);
    out << s.Time() << ',' << motion.position << ',' << motion.speed << ',' << s.Iterations() << ',' << s.ForceResidual() << ',' << s.InternalSteps();
    const auto& state = s.VehicleState();
    for (int i = 0; i < 10; ++i)
        out << ',' << state.q[i] << ',' << state.v[i] << ',' << state.a[i];
    for (const auto& point : s.Contacts()) {
        const auto response = s.RailResponse(point.rail_id, point.x);
        const auto input = railway::TrackChannelsAtPosition(p, point.x, 0);
        const double value = c.rail_count == 1 ? (input.displacement[0] + input.displacement[1]) / 2 : input.displacement[point.rail_id];
        const double slope = c.rail_count == 1 ? (input.slope[0] + input.slope[1]) / 2 : input.slope[point.rail_id];
        out << ',' << point.x << ',' << point.force << ',' << point.gap << ',' << point.penetration << ',' << (point.state == ContactState::Contact ? 1 : 0) << ','
            << response.displacement.z() << ',' << response.velocity.z() << ',' << response.acceleration.z() << ',' << value << ',' << slope * motion.speed;
    }
    out << '\n';
}
}  // namespace

int main(int argc, char* argv[]) {
    fs::path output;
    bool created_output = false;
    try {
        std::string input = (fs::absolute(argv[0]).parent_path() / "config/coupled_vertical.json").string();
        // Read --config first so every subsequent override is applied consistently.
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--help") {
                std::cout << "railway_coupled --config FILE --output NEW_DIRECTORY [--dt S] [--duration S] [--modes N]\n"
                             "  [--backend modal|fem] [--elements N] [--rails 1|2] [--speed M/S] [--flat]\n";
                return 0;
            }
            if (option == "--config") {
                if (++i == argc)
                    throw std::invalid_argument("Missing config path");
                input = argv[i];
            }
        }
        auto c = SimulationConfig::Load(input);
        bool flat = false;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--flat") {
                flat = true;
                continue;
            }
            if (++i == argc)
                throw std::invalid_argument("Missing value for " + option);
            const std::string value = argv[i];
            if (option == "--config")
                continue;
            if (option == "--output")
                output = value;
            else if (option == "--dt")
                c.dt = Number(value);
            else if (option == "--duration")
                c.duration = Number(value);
            else if (option == "--modes")
                c.rail.modes = Integer(value);
            else if (option == "--elements")
                c.rail.elements = Integer(value);
            else if (option == "--rails")
                c.rail_count = Integer(value);
            else if (option == "--speed")
                c.speed.points = {{0, Number(value)}};
            else if (option == "--backend") {
                if (value != "modal" && value != "fem")
                    throw std::invalid_argument("Unknown backend");
                c.fem = value == "fem";
            } else
                throw std::invalid_argument("Unknown option: " + option);
        }
        c.Validate();
        if (output.empty())
            throw std::invalid_argument("--output NEW_DIRECTORY is required");
        if (fs::exists(output))
            throw std::invalid_argument("Output directory already exists");
        auto p = railway::Parameters::Load(c.vehicle_config);
        if (p.use_3d)
            throw std::invalid_argument("Use a vertical vehicle config; 3D parameters are not part of this baseline");
        if (flat)
            p.track_enabled = false;
        const auto setup_start = std::chrono::steady_clock::now();
        CoupledSimulation simulation(p, c);
        const double setup_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - setup_start).count();
        fs::create_directories(output);
        created_output = true;
        Snapshot(input, output, c, p);
        std::ofstream response(output / "response.csv"), event_file(output / "events.csv");
        response << std::setprecision(16);
        event_file << std::setprecision(16) << "time_s,wheel_id,rail_id,from_contact,to_contact\n";
        Header(response, c.rail_count);
        Row(response, simulation, c, p);
        const auto start = std::chrono::steady_clock::now();
        const auto cpu_start = std::clock();
        size_t written_events = 0;
        double max_residual = 0;
        int max_iterations = 0;
        const int steps = static_cast<int>(std::llround(c.duration / c.dt));
        for (int i = 0; i < steps; ++i) {
            simulation.Step(c.dt);
            Row(response, simulation, c, p);
            max_residual = std::max(max_residual, simulation.ForceResidual());
            max_iterations = std::max(max_iterations, simulation.Iterations());
            while (written_events < simulation.Events().size()) {
                const auto& e = simulation.Events()[written_events++];
                event_file << e.time << ',' << e.wheel_id << ',' << e.rail_id << ',' << (e.from == ContactState::Contact) << ',' << (e.to == ContactState::Contact) << '\n';
            }
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const double cpu_seconds = static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
        response.flush();
        event_file.flush();
        if (!response || !event_file)
            throw std::runtime_error("Result output failed");
        std::ofstream summary(output / "summary.txt");
        summary << std::setprecision(12) << "status=complete\nmodel=10DOF vertical Hertz coupled\nbackend=" << (c.fem ? "fem" : "modal") << "\nrail_count=" << c.rail_count
                << "\nstatic_relative_residual=" << simulation.StaticResidual() << "\nmax_force_relative_residual=" << max_residual
                << "\nmax_coupling_iterations=" << max_iterations << "\ninternal_steps=" << simulation.InternalSteps() << "\nminimum_internal_dt_s=" << simulation.MinInternalDt()
                << "\nmax_phase_increment=" << c.max_phase_increment << "\ncontact_events=" << written_events << "\nsetup_wall_seconds=" << setup_seconds
                << "\nsimulation_and_output_wall_seconds=" << seconds << "\nprocess_clock_seconds=" << cpu_seconds << "\nreal_time_factor=" << seconds / c.duration
                << "\nparameter_status=Illustrative rail/Hertz/inertia; source damper initial tangents; not experimentally validated\n";
        summary.flush();
        if (!summary)
            throw std::runtime_error("Summary output failed");
        std::cout << "Completed " << steps << " steps; max iterations=" << max_iterations << "; output=" << output.string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        if (created_output) {
            std::ofstream failure(output / "summary.txt");
            failure << "status=failed\nerror=" << error.what() << '\n';
        }
        std::cerr << error.what() << '\n';
        return 1;
    }
}

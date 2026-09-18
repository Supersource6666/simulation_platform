// Spatial 27-DOF model checks: equilibrium drift, axle delays, response magnitude.
// Independent of Chrono dynamic integration so it can be run alongside the legacy track tests.
#include "Parameters.h"
#include "Track.h"
#include "VehicleModel3D.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
void Check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "Need a configuration path");
        auto p = railway::Parameters::Load(argv[1]);
        Check(p.use_3d, "Configuration does not enable the 3-D model");
        // 1. Flat equilibrium: lateral, vertical and angular drift must stay tiny.
        auto flat = p;
        flat.amplitude = 0;
        flat.track_enabled = false;
        railway::VehicleModel3D eq(flat);
        for (int i = 0; i < 1500; ++i) {
            eq.Step(0.001);
            const auto cs = eq.Car();
            Check(std::isfinite(cs.z) && std::abs(cs.z) < 1e-6, "Carbody z drift too large");
            Check(std::isfinite(cs.y) && std::abs(cs.y) < 1e-6, "Carbody y drift too large");
            Check(std::isfinite(cs.pitch) && std::abs(cs.pitch) < 1e-6, "Carbody pitch drift too large");
            Check(std::isfinite(cs.roll) && std::abs(cs.roll) < 1e-6, "Carbody roll drift too large");
            Check(std::isfinite(cs.yaw) && std::abs(cs.yaw) < 1e-6, "Carbody yaw drift too large");
        }
        const double weight = (p.car_mass + 2 * p.bogie_mass + 4 * p.wheelset_mass) * p.gravity;
        Check(std::abs(eq.SupportForceSum() - weight) < 1e-3, "Static support forces do not balance weight");
        // 2. Track delay: the wheelset at axle index i should see the same sample as the leading axle
        //    after time shift of (x[0]-x[i]) / speed.
        Check(p.track_data || p.amplitude > 0, "Dynamic test requires either track file or synthetic amplitude");
        const double time = (p.track_start + 0.37 * p.track_length) / p.speed;
        const auto front = railway::TrackChannels(p, time, 0);
        for (double delay : {0.0, p.wheelbase, p.bogie_spacing, p.bogie_spacing + p.wheelbase}) {
            const auto following = railway::TrackChannels(p, time + delay / p.speed, delay);
            for (int j = 0; j < 4; ++j)
                Check(std::abs(front.displacement[j] - following.displacement[j]) < 1e-11,
                      "Wheelset delay check failed");
        }
        // 3. Vertical response must be non-zero under sinusoidal track input.
        railway::VehicleModel3D dyn(p);
        double peak_z = 0;
        const double end = (p.track_start + p.track_length + p.bogie_spacing + p.wheelbase) / p.speed + 1;
        const int count = static_cast<int>(std::ceil(end / 0.001));
        Check(count <= 100000, "Dynamic test scenario too long");
        for (int i = 0; i < count; ++i) {
            dyn.Step(0.001);
            const auto c = dyn.Car();
            Check(std::isfinite(c.z) && std::isfinite(c.y) && std::isfinite(c.yaw),
                  "Non-finite state in 3D dynamic step");
            Check(std::abs(c.z) < 0.1 && std::abs(c.y) < 0.05 && std::abs(c.yaw) < 0.05,
                  "3D state outside small-motion range");
            peak_z = std::max(peak_z, std::abs(c.z));
        }
        Check(peak_z > 1e-8, "Carbody does not respond to vertical track excitation");
        std::cout << "PASS(3D): equilibrium drift < 1um; static support=" << eq.SupportForceSum()
                  << " N; peak carbody z=" << peak_z << " m\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
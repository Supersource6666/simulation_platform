// Diagnostic matrix: measure the pure (unforced) instability growth of the car lateral mode
// by disabling track excitation and watching the exponential growth rate, with toggles to
// isolate the coupling that feeds it.
#include "Parameters.h"
#include "VehicleModel3D.h"
#include <cstring>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string mode = argc >= 3 ? argv[2] : "amp0";
    auto p = railway::Parameters::Load(argc >= 2 ? argv[1] :
        "applications/railway_vehicle/config/25t_yz_loaded_3d.json");
    p.amplitude = 0;  // default: pure stability measurement
    bool forced = (mode == "forced" || mode == "damper_4x" || mode == "damper_8x" || mode == "damper_flat" || mode == "support_soft" || mode == "longrun");
    if (forced) p.amplitude = 0.001;
    if (mode == "amp0") { /* baseline unforced */ }
    else if (mode == "lat_c10") { p.secondary_lat_c *= 10; }
    else if (mode == "lat_c0") { p.secondary_lat_c = 0; }
    else if (mode == "no_yaw_sec") { p.yaw_secondary_k = 0; p.yaw_secondary_c = 0; }
    else if (mode == "no_roll_sec") { p.roll_secondary_k = 0; p.roll_secondary_c = 0; }
    else if (mode == "no_yaw_pri") { p.yaw_primary_k = 0; p.yaw_primary_c = 0; }
    else if (mode == "yaw_off_pitch_on") { p.yaw_primary_k = 0; p.yaw_primary_c = 0; p.primary_pitch_k = 4.0e7; p.primary_pitch_c = 5.0e4; }
    else if (mode == "yaw_on_pitch_off") { p.primary_pitch_k = 0; p.primary_pitch_c = 0; }
    else if (mode == "no_pitch_pri") { p.primary_pitch_k = 0; p.primary_pitch_c = 0; }
    else if (mode == "pitch_soft") { p.primary_pitch_k = 4.0e6; p.primary_pitch_c = 5.0e4; }
    else if (mode == "pitch_damp") { p.primary_pitch_k = 4.0e7; p.primary_pitch_c = 5.0e6; }
    else if (mode == "pk1e6") { p.primary_pitch_k = 1.0e6; p.primary_pitch_c = 1.0e4; }
    else if (mode == "pk2e6") { p.primary_pitch_k = 2.0e6; p.primary_pitch_c = 2.0e4; }
    else if (mode == "pk8e6") { p.primary_pitch_k = 8.0e6; p.primary_pitch_c = 8.0e4; }
    else if (mode == "pk15e6") { p.primary_pitch_k = 1.5e7; p.primary_pitch_c = 1.5e5; }
    else if (mode == "pk30e6") { p.primary_pitch_k = 3.0e7; p.primary_pitch_c = 3.0e5; }
    else if (mode == "no_roll_pri") { p.roll_primary_k = 0; p.roll_primary_c = 0; }
    else if (mode == "no_creep") { p.creep_lat = 0; p.creep_spin = 0; p.wheel_conicity = 0; }
    else if (mode == "f22_0") { p.creep_lat = 0; }
    else if (mode == "f22_small") { p.creep_lat = 5.5e4; }
    else if (mode == "spin_0") { p.creep_spin = 0; }
    else if (mode == "conicity_0") { p.wheel_conicity = 0; }
    else if (mode == "no_lat_k") { p.lateral_contact_k = 0; }
    else if (mode == "speed_low") { p.speed = 10.0; }
    else if (mode == "speed_0") { p.speed = 0.001; }
    else if (mode == "no_pri_lat") { p.primary_lat_k = 0; p.primary_lat_c = 0; }
    else if (mode == "damper_4x") { p.secondary_damper_count *= 2; p.primary_damper_count *= 2; }
    else if (mode == "damper_8x") { p.secondary_damper_count *= 4; p.primary_damper_count *= 4; }
    else if (mode == "damper_3x") { p.secondary_damper_count *= 1.5; p.primary_damper_count *= 1.5; }
    else if (mode == "damper_flat") {
        railway::DamperPoint dp0{0.0, 0.0};
        p.secondary_curve.assign(2, dp0);
        railway::DamperPoint dpmax{1.5, 30000.0};
        p.secondary_curve.push_back(dpmax);
        p.secondary_damper_count *= 2;
    }
    else if (mode == "support_soft") { p.support_k = 1.0e7; p.support_c = 5000; }
    else if (mode == "support_noisy") { p.support_c = 0; }
    else if (mode == "pitch_off") { p.primary_pitch_k = 0; p.primary_pitch_c = 0; }
    else if (mode == "sec_stiff_off") { p.secondary_k = 0; }
    std::cout << "mode=" << mode << "\n";
    railway::VehicleModel3D m3(p);
    const double dt = 0.001;
    const int nsteps = (mode == "longrun") ? 15000 : 8830;
    for (int s = 0; s < nsteps; ++s) {
        m3.Step(dt);
        if (s % 10 == 0) {
            const auto c = m3.Car();
            const auto b0 = m3.Bogie(0);
            const auto b1 = m3.Bogie(1);
            const auto ws = m3.Wheelset(0);
            std::cout << "  t=" << m3.Time()
                      << " car y=" << c.y << " z=" << c.z
                      << " b0 z=" << b0.z << " b1 z=" << b1.z
                      << " ws0 z=" << ws.z
                      << " minsup=" << m3.MinSupportForce() << "\n";
        }
    }
    std::cout << "  Done\n";
    return 0;
}

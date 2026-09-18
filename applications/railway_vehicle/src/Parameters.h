#pragma once
#include <string>
#include <vector>
#include "DamperCurve.h"
#include "TrackData.h"

namespace railway {
struct Parameters {
    std::string model_label, parameter_status;
    std::string track_file;
    std::shared_ptr<const TrackData> track_data;
    double track_lead_in = 5;
    bool track_enabled = true;
    double car_mass, bogie_mass, wheelset_mass;
    double car_height, bogie_height, wheel_radius;
    double primary_lower_z, primary_upper_z, secondary_lower_z, secondary_upper_z;
    double bogie_spacing, wheelbase;
    double primary_k, primary_c, secondary_k, secondary_c, support_k, support_c;
    double primary_damper_count = 0, secondary_damper_count = 0;
    std::vector<DamperPoint> primary_curve, secondary_curve;
    double speed, amplitude, wavelength, track_start, track_length, gravity;

    // ---- 3D spatial dynamics extensions (optional; defaults reproduce vertical-only behaviour) ----
    // Diagonal inertia tensors expressed in body-local COM frame (kg*m^2).
    // Off-diagonal products stay zero; reference axes match the documented coordinate convention.
    struct Inertia3D {
        double Ixx = 0, Iyy = 0, Izz = 0;
    };
    Inertia3D car_inertia, bogie_inertia, wheelset_inertia;
    // Suspension anchor offsets in body-local frames (m). Used to map body-frame motion to
    // spring/damper endpoints at primary/secondary attachment points.
    double primary_y_offset = 0.96;          // Lateral axlebox position relative to wheelset COM.
    double secondary_y_offset = 0.96;        // Lateral air-spring anchor relative to bogie/carbody COM.
    // Linear lateral suspension (N/m) and damping (N*s/m). One number per suspension stage, summed over
    // the symmetric pair; the force functors apply the sum to a single representative TSDA on each side.
    double primary_lat_k = 1.3e7, primary_lat_c = 1.0e4;
    double secondary_lat_k = 3.6e5, secondary_lat_c = 2.0e4;
    // Anti-hunting yaw damper + yaw spring constants (each end: carbody<->bogie, bogie<->wheelset pair).
    double yaw_secondary_k = 5.0e6;          // N*m/rad between carbody and bogie, restoring yaw alignment
    double yaw_secondary_c = 4.0e5;          // N*m*s/rad, anti-hunting yaw damper (per bogie)
    double yaw_primary_k = 4.0e6;            // N*m/rad between bogie and wheelset pair (yaw restraint)
    double yaw_primary_c = 5.0e3;            // N*m*s/rad (small primary yaw damping)
    // Carbody<->bogie pitch restraint (anti-pitch). The pitch stiffness is dominated by the
    // secondary suspension geometry (longitudinal bogie spacing), so this RSDA adds a modest
    // stiffness plus the anti-pitch damper rate; it must NOT reuse the yaw coefficients.
    double secondary_pitch_k = 2.0e6;        // N*m/rad between carbody and bogie, pitch alignment
    double secondary_pitch_c = 2.0e6;        // N*m*s/rad, anti-pitch damper (per bogie)
    // Wheelset<->bogie pitch restraint (near-rigid traction pin). Negative sentinel means
    // "derive from yaw_primary_k/c * 10" which reproduces the historical behaviour.
    double primary_pitch_k = -1.0;           // N*m/rad, <0 -> yaw_primary_k * 10
    double primary_pitch_c = -1.0;           // N*m*s/rad, <0 -> yaw_primary_c * 10
    // Roll stiffness/damping applied at secondary stage (anti-roll bar equivalent), N*m/rad and N*m*s/rad.
    double roll_secondary_k = 1.2e6, roll_secondary_c = 1.0e4;
    double roll_primary_k = 6.0e5, roll_primary_c = 5.0e3;
    // Wheel-rail lateral contact: linear creep + conicity-based gravitational restoring force.
    double wheel_conicity = 0.05;            // Effective tread conicity (dimensionless), LM-type.
    double rolling_gauge = 1.493;            // Rolling circle lateral span 2b [m], default 25T LM tread.
    double creep_lat = 5.5e6;                // Kalker linear lateral creep coefficient f22 [N].
    double creep_long = 1.0e7;               // Kalker longitudinal creep coefficient f11 [N].
    double creep_spin = 6.0e5;               // Kalker spin creep coefficient f33 [N*m].
    double lateral_contact_k = 8.0e7;        // Lateral wheel-rail contact stiffness [N/m], very stiff.
    double track_curvature = 0.0;            // 1/R of the host track [1/m], 0 == straight.
    double track_cant = 0.0;                 // Superelevation angle [rad], positive raises outer rail.
    // Per-axle support L/R stiffness mismatch (linearised bilateral vertical contact, kept from 7-DOF).
    double support_lr_balance = 1.0;         // Multiplier on (Lv-Rv)/2 to distribute asymmetric vertical load.
    bool use_3d = false;                     // Run spatial model when true; false keeps legacy 7-DOF path.

    static Parameters Load(const std::string& path);
};
}
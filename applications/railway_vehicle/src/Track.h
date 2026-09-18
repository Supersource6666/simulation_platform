#pragma once
#include "Parameters.h"
#include <cmath>
#include <utility>

namespace railway {
// Four channel (Lv, Rv, Ld, Rd) PCHIP/C1 sample at the wheelset whose front axle is `axle_delay_m` behind
// the leading axle. The mean vertical channel is used to excite vertical dynamics and the differential
// (Ld-Rd) is reused by the 3-D lateral/hunting extension.
inline TrackSample TrackChannelsAtPosition(const Parameters& p, double leading_position_m, double axle_delay_m) {
    TrackSample sample;
    const double relative_s = leading_position_m - axle_delay_m - p.track_start;
    sample.s = relative_s + (p.track_data ? p.track_data->First() : 0);
    if (!p.track_enabled) return sample;
    if (p.track_data) {
        if (sample.s >= p.track_data->First()) return p.track_data->Sample(sample.s);
        // Synthetic C1 lead-in before the first measured point only. Never alter measured samples.
        if (relative_s <= -p.track_lead_in) return sample;
        const auto first = p.track_data->Sample(p.track_data->First());
        const double length = p.track_lead_in, u = (relative_s + length) / length;
        for (int j = 0; j < 4; ++j) {
            const double y = first.displacement[j], m = first.slope[j];
            sample.displacement[j] = (-2*u*u*u+3*u*u)*y + (u*u*u-u*u)*length*m;
            sample.slope[j] = (-6*u*u+6*u)*y/length + (3*u*u-2*u)*m;
        }
        return sample;
    }
    if (relative_s <= 0 || relative_s >= p.track_length) return sample;
    constexpr double pi = 3.14159265358979323846;
    const double a = pi * relative_s / p.track_length, b = 2 * pi * relative_s / p.wavelength;
    const double window = std::pow(std::sin(a), 2);
    const double y = p.amplitude * window * std::sin(b);
    const double slope = p.amplitude * ((pi/p.track_length)*std::sin(2*a)*std::sin(b) +
                                       window*(2*pi/p.wavelength)*std::cos(b));
    sample.displacement = {y, y, 0, 0};
    sample.slope = {slope, slope, 0, 0};
    return sample;
}
inline TrackSample TrackChannels(const Parameters& p, double time, double axle_delay_m) {
    return TrackChannelsAtPosition(p, p.speed * time, axle_delay_m);
}
inline std::pair<double, double> TrackInput(const Parameters& p, double time, double axle_delay_m) {
    const auto sample = TrackChannels(p, time, axle_delay_m);
    // Equal left/right support properties and suppressed roll => exact net vertical reduction.
    return {(sample.displacement[0] + sample.displacement[1]) / 2,
            p.speed * (sample.slope[0] + sample.slope[1]) / 2};
}
// Left and right vertical inputs separately, applied at -rolling_gauge/2 and +rolling_gauge/2.
// When the track file carries distinct Lv/Rv columns, vertical asymmetry produces a roll moment.
struct WheelRailVertical {
    double z_left = 0, z_right = 0;
    double vz_left = 0, vz_right = 0;
    double roll_moment = 0;  // vertical contribution to wheelset roll moment [N*m] (positive = roll right).
};
inline WheelRailVertical TrackVerticalSplit(const Parameters& p, double time, double axle_delay_m,
                                           double rolling_gauge, double static_axle_load_N) {
    WheelRailVertical out;
    const auto s = TrackChannels(p, time, axle_delay_m);
    out.z_left  = s.displacement[0];
    out.z_right = s.displacement[1];
    out.vz_left  = p.speed * s.slope[0];
    out.vz_right = p.speed * s.slope[1];
    // Antisymmetric vertical excitation enters the wheelset roll moment arm equal to half the gauge.
    const double half = rolling_gauge / 2;
    out.roll_moment = static_axle_load_N * (out.z_left - out.z_right) / (2 * half);
    return out;
}
// Lateral track input used by the 3-D lateral/hunting extension. The reported lateral irregularity is
// the mean of the two rails; differential alignment and cant are reported for diagnostic export.
struct WheelRailLateral {
    double y_track = 0;       // Average lateral irregularity of the two rails [m].
    double vy_track = 0;      // Time derivative [m/s], positive = positive y direction.
    double alignment = 0;     // (Ld - Rd)/2, effective lateral offset of the contact midpoint [m].
    double cant_rad = 0;      // Track cant contribution to lateral gravitational component.
};
inline WheelRailLateral TrackLateralInput(const Parameters& p, double time, double axle_delay_m) {
    WheelRailLateral out;
    const auto s = TrackChannels(p, time, axle_delay_m);
    out.y_track = (s.displacement[2] + s.displacement[3]) / 2;
    out.vy_track = p.speed * (s.slope[2] + s.slope[3]) / 2;
    out.alignment = (s.displacement[2] - s.displacement[3]) / 2;
    out.cant_rad = p.track_cant;
    return out;
}
}
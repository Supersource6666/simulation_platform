#pragma once
#include <cmath>
#include <vector>

namespace railway {
struct DamperPoint { double speed; double force; };

// Return signed resisting force for one damper. Input table is validated by Parameters::Load.
// Assume odd symmetry, add (0,0), interpolate linearly and extrapolate the last segment.
inline double DamperForce(const std::vector<DamperPoint>& points, double velocity) {
    const double speed = std::abs(velocity);
    size_t upper = 1;
    while (upper + 1 < points.size() && speed > points[upper].speed)
        ++upper;
    const auto& a = points[upper - 1];
    const auto& b = points[upper];
    const double force = a.force + (b.force - a.force) * (speed - a.speed) / (b.speed - a.speed);
    return std::copysign(force, velocity);
}
}

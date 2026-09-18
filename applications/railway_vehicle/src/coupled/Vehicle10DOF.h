#pragma once
#include "TrackDynamics.h"
#include "../Parameters.h"
#include <array>

namespace railway::coupled {
// [zc, theta_c, zb1, theta_b1, zb2, theta_b2, zw1..zw4].
// z upward; theta is nose-up slope, i.e. negative right-handed Y rotation.
class Vehicle10DOF {
  public:
    Vehicle10DOF(const Parameters& p, double car_pitch_inertia, double bogie_pitch_inertia);
    State Predict(const State& initial, const Vector& contact_load, double dt);
    State PredictPrescribedWheels(const State& initial, const State& wheels, double dt) const;
    Matrix compliance;
    Matrix mass, stiffness, damping;
    Vector gravity;
    std::array<double, 4> offsets;

  private:
    double cached_dt = -1;
    Eigen::LDLT<Matrix> factor;
};
}  // namespace railway::coupled

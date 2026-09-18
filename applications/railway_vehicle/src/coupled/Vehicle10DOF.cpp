#include "Vehicle10DOF.h"
#include <cmath>
#include <stdexcept>

namespace railway::coupled {
Vehicle10DOF::Vehicle10DOF(const Parameters& p, double car_pitch_inertia, double bogie_pitch_inertia)
    : mass(Matrix::Zero(10, 10)), stiffness(mass), damping(mass), gravity(Vector::Zero(10)) {
    for (double x : {p.car_mass, p.bogie_mass, p.wheelset_mass, car_pitch_inertia, bogie_pitch_inertia, p.primary_k, p.secondary_k, p.bogie_spacing, p.wheelbase, p.gravity})
        if (!std::isfinite(x) || x <= 0)
            throw std::invalid_argument("Invalid 10DOF mass/inertia/geometry/stiffness");
    const double a = p.bogie_spacing / 2, b = p.wheelbase / 2;
    offsets = {a + b, a - b, -a + b, -a - b};
    mass.diagonal() << p.car_mass, car_pitch_inertia, p.bogie_mass, bogie_pitch_inertia, p.bogie_mass, bogie_pitch_inertia, p.wheelset_mass, p.wheelset_mass, p.wheelset_mass,
        p.wheelset_mass;
    for (int i : {0, 2, 4, 6, 7, 8, 9})
        gravity[i] = -mass(i, i) * p.gravity;
    // Linear validation baseline: use each source damper's initial tangent.
    const double cp = p.primary_curve.empty() ? p.primary_c : p.primary_damper_count * p.primary_curve[1].force / p.primary_curve[1].speed;
    const double cs = p.secondary_curve.empty() ? p.secondary_c : p.secondary_damper_count * p.secondary_curve[1].force / p.secondary_curve[1].speed;
    if (!std::isfinite(cp) || !std::isfinite(cs) || cp < 0 || cs < 0)
        throw std::invalid_argument("Invalid suspension damping");
    auto spring = [&](const Vector& bvec, double k, double c) {
        stiffness += k * bvec * bvec.transpose();
        damping += c * bvec * bvec.transpose();
    };
    for (int j = 0; j < 2; ++j) {
        Vector bvec = Vector::Zero(10);
        bvec[0] = 1;
        bvec[1] = j == 0 ? a : -a;
        bvec[2 + 2 * j] = -1;
        spring(bvec, p.secondary_k, cs);
        for (int axle = 0; axle < 2; ++axle) {
            bvec.setZero();
            bvec[2 + 2 * j] = 1;
            bvec[3 + 2 * j] = axle == 0 ? b : -b;
            bvec[6 + 2 * j + axle] = -1;
            spring(bvec, p.primary_k, cp);
        }
    }
}

State Vehicle10DOF::Predict(const State& initial, const Vector& contact_load, double dt) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid vehicle timestep");
    if (dt != cached_dt) {
        factor.compute(stiffness + (4 / (dt * dt)) * mass + (2 / dt) * damping);
        if (factor.info() != Eigen::Success)
            throw std::runtime_error("Vehicle Newmark factorization failed");
        compliance = factor.solve(Matrix::Identity(10, 10));
        cached_dt = dt;
    }
    const Vector qp = initial.q + dt * initial.v + (dt * dt / 4) * initial.a;
    const Vector vp = initial.v + (dt / 2) * initial.a;
    State result(10);
    result.q = factor.solve(gravity + contact_load + (4 / (dt * dt)) * mass * qp + damping * ((2 / dt) * qp - vp));
    result.a = (4 / (dt * dt)) * (result.q - qp);
    result.v = vp + (dt / 2) * result.a;
    return result;
}

State Vehicle10DOF::PredictPrescribedWheels(const State& initial, const State& wheels, double dt) const {
    if (!std::isfinite(dt) || dt <= 0 || wheels.q.size() != 4 || wheels.v.size() != 4 || wheels.a.size() != 4)
        throw std::invalid_argument("Invalid prescribed-wheel state or timestep");
    const Matrix m = mass.topLeftCorner(6, 6), c = damping.topLeftCorner(6, 6), k = stiffness.topLeftCorner(6, 6);
    const Vector qp = initial.q.head(6) + dt * initial.v.head(6) + (dt * dt / 4) * initial.a.head(6);
    const Vector vp = initial.v.head(6) + (dt / 2) * initial.a.head(6);
    const Vector load = gravity.head(6) - stiffness.topRightCorner(6, 4) * wheels.q - damping.topRightCorner(6, 4) * wheels.v;
    State result(10);
    result.q.head(6) = (k + (4 / (dt * dt)) * m + (2 / dt) * c).ldlt().solve(load + (4 / (dt * dt)) * m * qp + c * ((2 / dt) * qp - vp));
    result.a.head(6) = (4 / (dt * dt)) * (result.q.head(6) - qp);
    result.v.head(6) = vp + (dt / 2) * result.a.head(6);
    result.q.tail(4) = wheels.q;
    result.v.tail(4) = wheels.v;
    result.a.tail(4) = wheels.a;
    return result;
}
}  // namespace railway::coupled

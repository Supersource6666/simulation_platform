#include "TrackDynamics.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>

namespace railway::coupled {
void RailParameters::Validate() const {
    for (double value : {length, mass_per_length, bending_stiffness})
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument("Rail length, rhoA and EI must be positive and finite");
    for (double value : {foundation_stiffness, foundation_damping})
        if (!std::isfinite(value) || value < 0)
            throw std::invalid_argument("Winkler coefficients must be finite and nonnegative");
    if (modes < 1 || modes > 1000 || elements < 2 || elements > 500)
        throw std::invalid_argument("Expected 1..1000 modes and 2..500 FEM elements");
}

TrackResponse ITrackDynamics::Evaluate(double x, const State& state) const {
    const Vector shape = Basis(x);
    TrackResponse response;
    response.displacement.z() = shape.dot(state.q);
    response.velocity.z() = shape.dot(state.v);
    response.acceleration.z() = shape.dot(state.a);
    // Right-handed rotation about Y: theta_y = -dw/dx.
    response.rotation.y() = -SlopeBasis(x).dot(state.q);
    response.angular_velocity.y() = -SlopeBasis(x).dot(state.v);
    return response;
}

State ITrackDynamics::StaticState(const Vector& load) const {
    State state(static_cast<int>(Mass().size()));
    state.q = load.cwiseQuotient(Stiffness());
    return state;
}

ModalRail::ModalRail(const RailParameters& parameters, bool fem) : p(parameters), use_fem(fem) {
    p.Validate();
    if (fem) {
        BuildFiniteElements();
        return;
    }
    mass = Vector::Constant(p.modes, p.mass_per_length * p.length / 2);
    damping = Vector::Constant(p.modes, p.foundation_damping * p.length / 2);
    stiffness.resize(p.modes);
    for (int n = 0; n < p.modes; ++n)
        stiffness[n] = (p.bending_stiffness * std::pow((n + 1) * pi / p.length, 4) + p.foundation_stiffness) * p.length / 2;
}

void ModalRail::BuildFiniteElements() {
    // Cubic Hermite EB elements with consistent mass and distributed foundation.
    // Eliminate only end translations; end rotations are free (zero moment).
    const int size = 2 * (p.elements + 1);
    const double h = p.length / p.elements;
    Matrix full_m = Matrix::Zero(size, size), full_k = full_m;
    Eigen::Matrix4d shape_integral, bending;
    shape_integral << 156, 22 * h, 54, -13 * h, 22 * h, 4 * h * h, 13 * h, -3 * h * h, 54, 13 * h, 156, -22 * h, -13 * h, -3 * h * h, -22 * h, 4 * h * h;
    shape_integral *= h / 420;
    bending << 12, 6 * h, -12, 6 * h, 6 * h, 4 * h * h, -6 * h, 2 * h * h, -12, -6 * h, 12, -6 * h, 6 * h, 2 * h * h, -6 * h, 4 * h * h;
    bending *= p.bending_stiffness / (h * h * h);
    for (int e = 0; e < p.elements; ++e) {
        full_m.block<4, 4>(2 * e, 2 * e) += p.mass_per_length * shape_integral;
        full_k.block<4, 4>(2 * e, 2 * e) += bending + p.foundation_stiffness * shape_integral;
    }
    std::vector<int> free;
    for (int i = 0; i < size; ++i)
        if (i != 0 && i != size - 2)
            free.push_back(i);
    const int count = static_cast<int>(free.size());
    Matrix m(count, count), k(count, count);
    for (int i = 0; i < count; ++i)
        for (int j = 0; j < count; ++j) {
            m(i, j) = full_m(free[i], free[j]);
            k(i, j) = full_k(free[i], free[j]);
        }
    Eigen::GeneralizedSelfAdjointEigenSolver<Matrix> solver(k, m);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= 0)
        throw std::runtime_error("FEM eigenproblem failed");
    // Retain ALL spatial FEM DOFs; modal diagonalization only accelerates time stepping.
    mass = Vector::Ones(count);
    stiffness = solver.eigenvalues();
    damping = Vector::Constant(count, p.foundation_damping / p.mass_per_length);
    eigenvectors = Matrix::Zero(size, count);
    for (int i = 0; i < count; ++i)
        eigenvectors.row(free[i]) = solver.eigenvectors().row(i);
}

Vector ModalRail::Shape(double x, bool derivative) const {
    if (!std::isfinite(x) || x < 0 || x > p.length)
        throw std::out_of_range("Contact/evaluation lies outside finite rail");
    if (!use_fem) {
        Vector shape(p.modes);
        for (int n = 0; n < p.modes; ++n) {
            const double k = (n + 1) * pi / p.length;
            shape[n] = derivative ? k * std::cos(k * x) : std::sin(k * x);
        }
        return shape;
    }
    const double h = p.length / p.elements;
    const int e = std::min(static_cast<int>(x / h), p.elements - 1);
    const double s = (x - e * h) / h;
    Eigen::Vector4d shape;
    if (derivative)
        shape << (-6 * s + 6 * s * s) / h, 1 - 4 * s + 3 * s * s, (6 * s - 6 * s * s) / h, -2 * s + 3 * s * s;
    else
        shape << 1 - 3 * s * s + 2 * s * s * s, h * (s - 2 * s * s + s * s * s), 3 * s * s - 2 * s * s * s, h * (-s * s + s * s * s);
    return eigenvectors.middleRows(2 * e, 4).transpose() * shape;
}

Vector ModalRail::Basis(double x) const {
    return Shape(x, false);
}
Vector ModalRail::SlopeBasis(double x) const {
    return Shape(x, true);
}

State ModalRail::Predict(const State& initial, const Vector& load_start, const Vector& load_end, double dt) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("dt must be finite and positive");
    const int count = static_cast<int>(mass.size());
    if (initial.q.size() != count || initial.v.size() != count || load_start.size() != count || load_end.size() != count)
        throw std::invalid_argument("Track state/load dimension mismatch");
    if (dt != cached_dt) {
        transitions.resize(count);
        for (int i = 0; i < count; ++i) {
            // Augmented state [q, v, Q, dQ/dt]. Matrix exponential handles
            // under-, critical and overdamping without separate singular formulas.
            Eigen::Matrix4d a = Eigen::Matrix4d::Zero();
            a(0, 1) = 1;
            a(1, 0) = -stiffness[i] / mass[i];
            a(1, 1) = -damping[i] / mass[i];
            a(1, 2) = 1 / mass[i];
            a(2, 3) = 1;
            transitions[i] = (a * dt).exp();
        }
        cached_dt = dt;
    }
    State out(count);
    for (int i = 0; i < count; ++i) {
        const Eigen::Vector4d initial_aug(initial.q[i], initial.v[i], load_start[i], (load_end[i] - load_start[i]) / dt);
        const Eigen::Vector4d next = transitions[i] * initial_aug;
        out.q[i] = next[0];
        out.v[i] = next[1];
        out.a[i] = (load_end[i] - damping[i] * next[1] - stiffness[i] * next[0]) / mass[i];
    }
    return out;
}
}  // namespace railway::coupled

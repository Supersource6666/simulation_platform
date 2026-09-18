#include "CoupledSimulation.h"
#include "../Track.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace railway::coupled {
CoupledSimulation::CoupledSimulation(const Parameters& p, const SimulationConfig& c)
    : parameters(p), config(c), vehicle(p, c.car_pitch_inertia, c.bogie_pitch_inertia), hertz{c.hertz_coefficient}, vehicle_state(10) {
    config.Validate();
    const double end = config.speed.Evaluate(config.duration, config.initial_position).position;
    if (config.initial_position + vehicle.offsets.back() <= 0 || end + vehicle.offsets.front() >= config.rail.length)
        throw std::invalid_argument("All axles must remain strictly inside the finite rail for the entire run");
    for (int j = 0; j < c.rail_count; ++j) {
        rails.push_back(std::make_unique<ModalRail>(c.rail, c.fem));
        rail_size = static_cast<int>(rails.back()->Mass().size());
        rail_states.emplace_back(rail_size);
    }
    forces = Vector::Zero(4 * c.rail_count);
    contacts.resize(static_cast<size_t>(forces.size()));
    InitializeStaticEquilibrium();
}

Matrix CoupledSimulation::ContactMap(double at_time, Vector& irregularity, std::vector<double>& positions) const {
    const int count = static_cast<int>(forces.size());
    Matrix b = Matrix::Zero(count, 10 + config.rail_count * rail_size);
    irregularity.resize(count);
    positions.resize(count);
    const double center = config.speed.Evaluate(at_time, config.initial_position).position;
    for (int c = 0; c < count; ++c) {
        const int rail = c / 4, wheel = c % 4;
        const double x = center + vehicle.offsets[wheel];
        if (x <= 0 || x >= config.rail.length)
            throw std::out_of_range("Wheel left the finite rail");
        positions[c] = x;
        b(c, 6 + wheel) = 1;
        b.block(c, 10 + rail * rail_size, 1, rail_size) = -rails[rail]->Basis(x).transpose();
        const auto input = TrackChannelsAtPosition(parameters, x, 0);
        irregularity[c] = config.rail_count == 1 ? (input.displacement[0] + input.displacement[1]) / 2 : input.displacement[rail];
    }
    return b;
}

void CoupledSimulation::InitializeStaticEquilibrium() {
    Vector irregularity;
    std::vector<double> positions;
    const Matrix b = ContactMap(0, irregularity, positions);
    const int size = static_cast<int>(b.cols()), count = static_cast<int>(b.rows());
    Matrix k = Matrix::Zero(size, size);
    k.topLeftCorner(10, 10) = vehicle.stiffness;
    Vector external = Vector::Zero(size);
    external.head(10) = vehicle.gravity;
    for (int j = 0; j < config.rail_count; ++j)
        k.block(10 + j * rail_size, 10 + j * rail_size, rail_size, rail_size).diagonal() = rails[j]->Stiffness();
    const double axle_load = -vehicle.gravity.sum() / 4;
    const double compression = std::pow(axle_load / (config.rail_count * hertz.coefficient), 2.0 / 3);
    Vector q = Vector::Zero(size);
    // Start below the highest rail at each axle so at least one contact supports
    // every wheelset, including measured inputs with large absolute offsets.
    for (int i = 0; i < 4; ++i) {
        double surface = irregularity[i];
        for (int j = 1; j < config.rail_count; ++j)
            surface = std::max(surface, irregularity[4 * j + i]);
        q[6 + i] = surface - compression;
    }
    const double primary_sag = (axle_load + vehicle.gravity[6]) / parameters.primary_k;
    q[2] = (q[6] + q[7]) / 2 - primary_sag;
    q[4] = (q[8] + q[9]) / 2 - primary_sag;
    q[3] = (q[6] - q[7]) / parameters.wheelbase;
    q[5] = (q[8] - q[9]) / parameters.wheelbase;
    q[0] = (q[2] + q[4]) / 2 + vehicle.gravity[0] / (2 * parameters.secondary_k);
    q[1] = (q[2] - q[4]) / parameters.bogie_spacing;
    auto evaluate = [&](const Vector& candidate, Vector& f, Vector& tangent) -> Vector {
        const Vector delta = irregularity - b * candidate;
        f.resize(count);
        tangent.resize(count);
        for (int i = 0; i < count; ++i) {
            f[i] = hertz.Force(delta[i]);
            tangent[i] = hertz.Tangent(delta[i]);
        }
        return k * candidate - external - b.transpose() * f;
    };
    Vector f, tangent;
    bool converged = false;
    for (int step = 0; step < 80; ++step) {
        const Vector r = evaluate(q, f, tangent);
        static_residual = r.lpNorm<Eigen::Infinity>() / std::max(external.norm(), config.force_reference);
        if (static_residual < 1e-11) {
            converged = true;
            break;
        }
        const Matrix jacobian = k + b.transpose() * tangent.asDiagonal() * b;
        const Vector change = jacobian.ldlt().solve(-r);
        if (!change.allFinite())
            throw std::runtime_error("Static equilibrium has singular support");
        double alpha = 1;
        bool accepted = false;
        for (int line = 0; line < 24; ++line) {
            Vector trial_f, trial_tangent;
            const Vector trial = q + alpha * change;
            if (evaluate(trial, trial_f, trial_tangent).norm() < r.norm()) {
                q = trial;
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!accepted)
            throw std::runtime_error("Static equilibrium line search failed");
    }
    if (!converged)
        throw std::runtime_error("Static Hertz/vehicle/rail equilibrium did not converge");
    vehicle_state = State(10);
    vehicle_state.q = q.head(10);
    for (int j = 0; j < config.rail_count; ++j) {
        rail_states[j] = State(rail_size);
        rail_states[j].q = q.segment(10 + j * rail_size, rail_size);
    }
    forces = f;
    time = 0;
    residual = 0;
    iterations = 0;
    internal_steps = 0;
    min_internal_dt = 0;
    events.clear();
    UpdateContacts(f, irregularity - b * q, positions, false);
}

void CoupledSimulation::UpdateContacts(const Vector& force, const Vector& compression, const std::vector<double>& positions, bool record) {
    for (int i = 0; i < force.size(); ++i) {
        auto& c = contacts[i];
        const auto next = compression[i] > 0 ? ContactState::Contact : ContactState::Separation;
        if (record && c.state != next)
            events.push_back({time, i % 4, i / 4, c.state, next});
        c.wheel_id = i % 4;
        c.rail_id = i / 4;
        c.x = positions[i];
        c.gap = -compression[i];
        c.penetration = std::max(0.0, compression[i]);
        c.force = force[i];
        c.state = next;
    }
}

double CoupledSimulation::StableStepEstimate() const {
    // Weyl bound for the mass-normalized tangent stiffness:
    // lambda_max(K0 + B^T H B) <= lambda_max(K0) + lambda_max(B^T H B).
    // The contact term has rank at most eight, so solve only that small eigenproblem.
    Vector irregularity;
    std::vector<double> positions;
    const Matrix b = ContactMap(time, irregularity, positions);
    Vector inverse_mass(b.cols());
    inverse_mass.head(10) = vehicle.mass.diagonal().cwiseInverse();
    const Vector scale = inverse_mass.head(10).cwiseSqrt();
    const Matrix normalized = scale.asDiagonal() * vehicle.stiffness * scale.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Matrix> vehicle_modes(normalized, Eigen::EigenvaluesOnly);
    double omega_squared = vehicle_modes.eigenvalues().maxCoeff();
    for (int j = 0; j < config.rail_count; ++j) {
        inverse_mass.segment(10 + j * rail_size, rail_size) = rails[j]->Mass().cwiseInverse();
        omega_squared = std::max(omega_squared, rails[j]->Stiffness().cwiseQuotient(rails[j]->Mass()).maxCoeff());
    }
    Vector tangent(forces.size());
    for (int i = 0; i < forces.size(); ++i)
        tangent[i] = std::sqrt(hertz.Tangent(contacts[i].penetration));
    const Matrix contact_mass = tangent.asDiagonal() * b * inverse_mass.asDiagonal() * b.transpose() * tangent.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Matrix> contact_modes(contact_mass, Eigen::EigenvaluesOnly);
    if (vehicle_modes.info() != Eigen::Success || contact_modes.info() != Eigen::Success)
        throw std::runtime_error("Cannot estimate coupling frequency bound");
    omega_squared += std::max(0.0, contact_modes.eigenvalues().maxCoeff());
    return config.max_phase_increment / std::sqrt(omega_squared);
}

void CoupledSimulation::Step(double dt) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid coupling timestep");
    // Macro-step transaction: an error after ANY internal step restores all public state.
    const State saved_vehicle = vehicle_state;
    const auto saved_rails = rail_states;
    const auto saved_contacts = contacts;
    const Vector saved_forces = forces;
    const size_t saved_events = events.size();
    const double saved_time = time, saved_residual = residual, saved_min_dt = min_internal_dt;
    const int saved_iterations = iterations, saved_steps = internal_steps;
    try {
        const double limit = StableStepEstimate();
        const double required = std::ceil(dt / limit);
        if (!std::isfinite(required) || required > 100000)
            throw std::runtime_error("Requested step needs too many internal steps");
        const int count = std::max(1, static_cast<int>(required));
        const double h = dt / count;
        double max_residual = 0;
        int max_iterations = 0;
        for (int i = 0; i < count; ++i) {
            // Recheck after impact, since the nonlinear Hertz tangent can increase.
            const double updated_limit = StableStepEstimate();
            const double refinement_count = std::ceil(h / updated_limit);
            if (!std::isfinite(refinement_count) || refinement_count > 1000)
                throw std::runtime_error("Contact frequency grew beyond substep budget");
            const int refinements = std::max(1, static_cast<int>(refinement_count));
            for (int j = 0; j < refinements; ++j) {
                Advance(h / refinements);
                ++internal_steps;
                min_internal_dt = min_internal_dt == 0 ? h / refinements : std::min(min_internal_dt, h / refinements);
                max_residual = std::max(max_residual, residual);
                max_iterations = std::max(max_iterations, iterations);
            }
        }
        time = saved_time + dt;
        residual = max_residual;
        iterations = max_iterations;
    } catch (...) {
        vehicle_state = saved_vehicle;
        rail_states = saved_rails;
        contacts = saved_contacts;
        forces = saved_forces;
        events.resize(saved_events);
        time = saved_time;
        residual = saved_residual;
        iterations = saved_iterations;
        internal_steps = saved_steps;
        min_internal_dt = saved_min_dt;
        throw;
    }
}

void CoupledSimulation::Advance(double dt) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid coupling timestep");
    Vector r0, r1;
    std::vector<double> x0, x1;
    const Matrix b0 = ContactMap(time, r0, x0), b1 = ContactMap(time + dt, r1, x1);
    const int count = static_cast<int>(forces.size());
    const int size = static_cast<int>(b1.cols());
    // Endpoint displacement is affine in the unknown endpoint contact forces.
    // Vehicle uses Newmark average acceleration; track uses exact FOH recurrence.
    Vector free_q = Vector::Zero(size);
    const State free_vehicle = vehicle.Predict(vehicle_state, Vector::Zero(10), dt);
    free_q.head(10) = free_vehicle.q;
    Matrix compliance = b1.leftCols(10) * vehicle.compliance * b1.leftCols(10).transpose();
    std::vector<Vector> start_loads;
    for (int j = 0; j < config.rail_count; ++j) {
        const auto br0 = b0.middleCols(10 + j * rail_size, rail_size);
        const auto br1 = b1.middleCols(10 + j * rail_size, rail_size);
        start_loads.push_back(br0.transpose() * forces);
        free_q.segment(10 + j * rail_size, rail_size) = rails[j]->Predict(rail_states[j], start_loads[j], Vector::Zero(rail_size), dt).q;
        const State unit = rails[j]->Predict(State(rail_size), Vector::Zero(rail_size), Vector::Ones(rail_size), dt);
        compliance += br1 * unit.q.asDiagonal() * br1.transpose();
    }
    const Vector free_compression = r1 - b1 * free_q;
    auto evaluate = [&](const Vector& f, Vector& tangent) -> Vector {
        const Vector delta = free_compression - compliance * f;
        Vector result(count);
        tangent.resize(count);
        for (int i = 0; i < count; ++i) {
            result[i] = f[i] - hertz.Force(delta[i]);
            tangent[i] = hertz.Tangent(delta[i]);
        }
        return result;
    };
    Vector f = forces;
    double displacement_change = 0, force_residual = 0;
    int used = 0;
    bool converged = false;
    for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
        used = iteration + 1;
        Vector tangent;
        const Vector r = evaluate(f, tangent);
        force_residual = r.norm() / std::max(f.norm(), config.force_reference);
        if (force_residual < config.force_tolerance && displacement_change < config.displacement_tolerance) {
            converged = true;
            break;
        }
        const Matrix jacobian = Matrix::Identity(count, count) + tangent.asDiagonal() * compliance;
        const Vector change = jacobian.fullPivLu().solve(-r);
        if (!change.allFinite())
            throw std::runtime_error("Nonfinite contact correction; step was not committed");
        double alpha = 1;
        bool accepted = false;
        for (int line = 0; line < 24; ++line) {
            const Vector trial = (f + alpha * change).cwiseMax(0.0);
            Vector trial_tangent;
            const double next_norm = evaluate(trial, trial_tangent).norm();
            if (next_norm < r.norm() || next_norm < 1e-9) {
                displacement_change = (compliance * (trial - f)).lpNorm<Eigen::Infinity>();
                f = trial;
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!accepted)
            throw std::runtime_error("Contact corrector line search failed; reduce dt. Step was not committed");
    }
    if (!converged)
        throw std::runtime_error("Coupling did not converge at t=" + std::to_string(time) + "; residual=" + std::to_string(force_residual) +
                                 ". Reduce dt or raise iteration limit; step was not committed");
    State next_vehicle = vehicle.Predict(vehicle_state, b1.leftCols(10).transpose() * f, dt);
    std::vector<State> next_rails;
    for (int j = 0; j < config.rail_count; ++j)
        next_rails.push_back(rails[j]->Predict(rail_states[j], start_loads[j], b1.middleCols(10 + j * rail_size, rail_size).transpose() * f, dt));
    if (!next_vehicle.q.allFinite() || !next_vehicle.v.allFinite() || !next_vehicle.a.allFinite())
        throw std::runtime_error("Nonfinite vehicle state; step was not committed");
    for (const auto& s : next_rails)
        if (!s.q.allFinite() || !s.v.allFinite() || !s.a.allFinite())
            throw std::runtime_error("Nonfinite rail state; step was not committed");
    vehicle_state = std::move(next_vehicle);
    rail_states = std::move(next_rails);
    forces = f;
    time += dt;
    iterations = used;
    residual = force_residual;
    UpdateContacts(f, free_compression - compliance * f, x1, true);
}
}  // namespace railway::coupled

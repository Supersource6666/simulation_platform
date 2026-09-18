#include "coupled/CoupledSimulation.h"
#include "Track.h"
#include <cmath>
#include <complex>
#include <iostream>
#include <stdexcept>

using namespace railway::coupled;
namespace {
void Check(bool ok, const char* message) {
    if (!ok)
        throw std::runtime_error(message);
}
template <class Function>
void Reject(Function function, const char* message) {
    bool failed = false;
    try {
        function();
    } catch (const std::exception&) {
        failed = true;
    }
    Check(failed, message);
}
void BeamChecks() {
    RailParameters p;
    p.length = 20;
    p.foundation_stiffness = 0;
    p.foundation_damping = 0;
    p.modes = 80;
    p.elements = 40;
    ModalRail rail(p), fem(p, true);
    const double load = -1e4, center = p.length / 2;
    const double analytical = load * std::pow(p.length, 3) / (48 * p.bending_stiffness);
    const State stat = rail.StaticState(load * rail.Basis(center));
    Check(std::abs(rail.Evaluate(center, stat).displacement.z() / analytical - 1) < 1e-6, "Modal static beam closed-form mismatch");
    const State fem_stat = fem.StaticState(load * fem.Basis(center));
    Check(std::abs(fem.Evaluate(center, fem_stat).displacement.z() / analytical - 1) < 1e-8, "FEM static beam closed-form mismatch");
    Check(std::abs(rail.Evaluate(0, stat).displacement.z()) < 1e-12 && std::abs(rail.Evaluate(p.length, stat).displacement.z()) < 1e-12, "Simply supported boundary failed");
    const Vector f1 = load * rail.Basis(4), f2 = 2 * load * rail.Basis(13);
    Check((rail.StaticState(f1 + f2).q - rail.StaticState(f1).q - rail.StaticState(f2).q).norm() < 1e-12, "Multiple contact load superposition failed");
    p.modes = 1;
    ModalRail single(p);
    const double omega = std::sqrt(single.Stiffness()[0] / single.Mass()[0]);
    const Vector f = Vector::Constant(1, load);
    const auto step = single.Predict(State(1), f, f, 0.01);
    Check(std::abs(step.q[0] - load / single.Stiffness()[0] * (1 - std::cos(omega * 0.01))) < 1e-12, "Exact oscillator constant-force transition failed");
    for (double ratio : {0.0, 1.0, 3.0}) {
        p.foundation_damping = 2 * ratio * omega * p.mass_per_length;
        ModalRail damped(p);
        const auto equilibrium = damped.StaticState(f);
        const auto held = damped.Predict(equilibrium, f, f, 0.013);
        Check((held.q - equilibrium.q).norm() < 1e-10 && held.v.norm() < 1e-10, "Damping regime static invariance failed");
    }
    // Constant-speed moving load has sinusoidal modal forcing and a closed form.
    const double velocity = 12, frequency = pi * velocity / p.length, dt = 1e-4;
    State moving(1);
    for (int i = 0; i < 1000; ++i)
        moving = single.Predict(moving, load * single.Basis(velocity * i * dt), load * single.Basis(velocity * (i + 1) * dt), dt);
    const double exact = load / single.Mass()[0] / (omega * omega - frequency * frequency) * (std::sin(frequency * 0.1) - frequency / omega * std::sin(omega * 0.1));
    Check(std::abs(moving.q[0] / exact - 1) < 1e-6, "Constant-speed moving-load analytical check failed");
    // Accelerating moving load: independent midpoint quadrature of Duhamel integral.
    auto position = [](double t) { return 2 + 3 * t + 0.5 * 8 * t * t; };
    State variable(1);
    for (int i = 0; i < 1000; ++i)
        variable = single.Predict(variable, load * single.Basis(position(i * dt)), load * single.Basis(position((i + 1) * dt)), dt);
    double reference = 0;
    const double h = 0.1 / 20000;
    for (int i = 0; i < 20000; ++i) {
        const double t = (i + 0.5) * h;
        reference += h * load * std::sin(pi * position(t) / p.length) * std::sin(omega * (0.1 - t)) / (single.Mass()[0] * omega);
    }
    Check(std::abs(variable.q[0] / reference - 1) < 2e-6, "Variable-speed load quadrature check failed");
    Reject([&] { single.Basis(-0.1); }, "Out-of-domain rail query accepted");
    std::cout << "beam static, FEM, exact recurrence, moving-load checks passed\n";
}

void VehicleChecks(const railway::Parameters& p) {
    Vehicle10DOF vehicle(p, 1.4067e6, 1216.8);
    Check((vehicle.stiffness - vehicle.stiffness.transpose()).norm() < 1e-8, "Vehicle stiffness not symmetric");
    Eigen::GeneralizedSelfAdjointEigenSolver<Matrix> frequencies(vehicle.stiffness.topLeftCorner(6, 6), vehicle.mass.topLeftCorner(6, 6));
    Check(frequencies.info() == Eigen::Success && frequencies.eigenvalues().minCoeff() > 0, "Fixed-wheel suspension modes invalid");
    std::cout << "fixed-wheel suspension frequencies Hz: " << (frequencies.eigenvalues().array().sqrt() / (2 * pi)).matrix().transpose() << '\n';
    // Exact harmonic particular solution is an independent frequency-domain reference.
    using Complex = std::complex<double>;
    const double omega = 7, dt = 1e-4;
    Vector force = Vector::Zero(10);
    force[6] = 1e4;
    const Eigen::MatrixXcd dynamic = vehicle.stiffness.cast<Complex>() - omega * omega * vehicle.mass.cast<Complex>() + Complex(0, omega) * vehicle.damping.cast<Complex>();
    const Eigen::VectorXcd amplitude = dynamic.fullPivLu().solve(force.cast<Complex>());
    State s(10);
    s.q = amplitude.real();
    s.v = -omega * amplitude.imag();
    s.a = -omega * omega * amplitude.real();
    vehicle.gravity.setZero();
    double max_error = 0;
    for (int i = 1; i <= 5000; ++i) {
        s = vehicle.Predict(s, force * std::cos(omega * i * dt), dt);
        const Vector expected = (amplitude * std::exp(Complex(0, omega * i * dt))).real();
        max_error = std::max(max_error, (s.q - expected).norm());
    }
    Check(max_error / amplitude.norm() < 2e-5, "Vehicle Newmark harmonic response mismatch");

    // Prescribed wheel displacement: wheel1=A*cos(omega*t), remaining wheels fixed.
    Vector wheel_amplitude = Vector::Zero(4);
    wheel_amplitude[0] = 1e-4;
    const Eigen::MatrixXcd prescribed_dynamic = dynamic.topLeftCorner(6, 6);
    const Eigen::VectorXcd drive =
        -(vehicle.stiffness.topRightCorner(6, 4).cast<Complex>() + Complex(0, omega) * vehicle.damping.topRightCorner(6, 4).cast<Complex>()) * wheel_amplitude.cast<Complex>();
    const Eigen::VectorXcd response = prescribed_dynamic.fullPivLu().solve(drive);
    State prescribed(10);
    prescribed.q.head(6) = response.real();
    prescribed.v.head(6) = -omega * response.imag();
    prescribed.a.head(6) = -omega * omega * response.real();
    prescribed.q.tail(4) = wheel_amplitude;
    prescribed.a.tail(4) = -omega * omega * wheel_amplitude;
    max_error = 0;
    for (int i = 1; i <= 5000; ++i) {
        State wheels(4);
        wheels.q = wheel_amplitude * std::cos(omega * i * dt);
        wheels.v = -omega * wheel_amplitude * std::sin(omega * i * dt);
        wheels.a = -omega * omega * wheels.q;
        prescribed = vehicle.PredictPrescribedWheels(prescribed, wheels, dt);
        const Vector expected = (response * std::exp(Complex(0, omega * i * dt))).real();
        max_error = std::max(max_error, (prescribed.q.head(6) - expected).norm());
    }
    Check(max_error / response.norm() < 2e-5, "Prescribed sinusoidal wheel response mismatch");
    std::cout << "10DOF harmonic force and prescribed-wheel checks passed\n";
}

void CouplingChecks(railway::Parameters p, SimulationConfig c) {
    p.track_enabled = false;
    c.speed.points = {{0, 0}};
    c.duration = 0.02;
    c.rail.modes = 20;
    c.rail_count = 2;
    CoupledSimulation static_run(p, c);
    Check(static_run.StaticResidual() < 1e-10, "Static residual exceeds tolerance");
    const Vector initial = static_run.VehicleState().q;
    double support = 0;
    for (const auto& point : static_run.Contacts())
        support += point.force;
    const double weight = (p.car_mass + 2 * p.bogie_mass + 4 * p.wheelset_mass) * p.gravity;
    Check(std::abs(support / weight - 1) < 1e-9, "Total static support does not balance gravity");
    for (int i = 0; i < 200; ++i)
        static_run.Step(1e-4);
    Check((static_run.VehicleState().q - initial).norm() < 1e-8, "Stationary coupled static drift");
    for (int i = 0; i < 4; ++i)
        Check(std::abs(static_run.Contacts()[i].force - static_run.Contacts()[i + 4].force) < 1e-5, "Symmetric twin rails diverged");
    c.speed.points = {{0, 20}, {0.01, 25}, {0.02, 30}};
    CoupledSimulation moving(p, c);
    for (int i = 0; i < 200; ++i)
        moving.Step(1e-4);
    Check(moving.ForceResidual() < c.force_tolerance, "Moving coupled force residual failed");
    for (const auto& point : moving.Contacts()) {
        const double physical = HertzContact{c.hertz_coefficient}.Force(point.penetration);
        Check(std::abs(point.force - physical) < 0.1, "Reported force inconsistent with Hertz compression");
    }

    auto fine_config = c;
    fine_config.rail.modes = 80;
    fine_config.rail_count = 1;
    CoupledSimulation fine(p, fine_config), coarse(p, fine_config);
    for (int i = 0; i < 200; ++i)
        fine.Step(1e-4);
    for (int i = 0; i < 20; ++i)
        coarse.Step(1e-3);
    Check(coarse.InternalSteps() > 20, "Large macro step did not subdivide");
    for (int i = 0; i < 4; ++i)
        Check(std::abs(coarse.Contacts()[i].force / fine.Contacts()[i].force - 1) < 1e-3, "Substepped coarse/fine force mismatch");
    // Large absolute irregularity offsets must not leave the Newton initial
    // guess unsupported; test the static problem before any dynamic excitation.
    auto offset_parameters = p;
    offset_parameters.track_enabled = true;
    offset_parameters.track_data.reset();
    offset_parameters.amplitude = 0.03;
    offset_parameters.track_start = 0;
    offset_parameters.track_length = 200;
    offset_parameters.wavelength = 50;
    auto offset_config = c;
    offset_config.speed.points = {{0, 0}};
    CoupledSimulation offset_equilibrium(offset_parameters, offset_config);
    Check(offset_equilibrium.StaticResidual() < 1e-10, "Offset-track static initialization failed");
    const Vector offset_initial = offset_equilibrium.VehicleState().q;
    for (int i = 0; i < 200; ++i)
        offset_equilibrium.Step(1e-4);
    Check((offset_equilibrium.VehicleState().q - offset_initial).norm() < 1e-8, "Offset-track equilibrium drift");

    c.max_iterations = 1;
    CoupledSimulation rejected(p, c);
    const Vector before = rejected.VehicleState().q;
    Reject([&] { rejected.Step(0.005); }, "Unconverged coupling step was accepted");
    Check(rejected.Time() == 0 && (rejected.VehicleState().q - before).norm() == 0 && rejected.Events().empty(), "Rejected step changed committed state");
    c.max_iterations = 15;
    c.speed.points = {{0, 33.333333333}};
    c.duration = 0.2;
    c.rail_count = 1;
    p.track_enabled = true;
    p.track_data.reset();
    p.amplitude = 0.015;
    p.wavelength = 1;
    p.track_start = 40;
    p.track_length = 8;
    CoupledSimulation impact(p, c);
    for (int i = 0; i < 2000; ++i)
        impact.Step(1e-4);
    bool separation = false, recontact = false;
    for (const auto& e : impact.Events()) {
        separation |= e.to == ContactState::Separation;
        recontact |= e.to == ContactState::Contact;
    }
    Check(separation && recontact, "Separation/recontact integration scenario did not exercise both events");
    for (const auto& point : impact.Contacts())
        Check(point.force >= 0 && (point.state == ContactState::Contact || point.force < 1e-6), "Tensile/separated contact force");
    std::cout << "static equilibrium, twin rail, variable speed, rollback, separation/recontact checks passed\n";
}
}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 2)
            throw std::runtime_error("Expected coupled config path");
        auto c = SimulationConfig::Load(argv[1]);
        auto p = railway::Parameters::Load(c.vehicle_config);
        HertzContact h{1e11};
        Check(h.Force(-1e-4) == 0 && h.Force(0) == 0 && std::abs(h.Force(1e-4) - 1e5) < 1e-8, "Hertz unilateral law failed");
        SpeedProfile speed;
        speed.points = {{0, 0}, {2, 4}, {3, 4}};
        speed.Validate();
        const auto m = speed.Evaluate(1, 10);
        Check(m.position == 11 && m.speed == 2 && m.acceleration == 2, "Accelerating speed integral failed");
        Check(speed.Evaluate(4, 10).position == 22, "Speed tail integration failed");
        const double x = p.track_start + 0.37 * p.track_length, dx = 1e-4;
        const auto sample = railway::TrackChannelsAtPosition(p, x, 0);
        const double derivative = (railway::TrackChannelsAtPosition(p, x + dx, 0).displacement[0] - railway::TrackChannelsAtPosition(p, x - dx, 0).displacement[0]) / (2 * dx);
        Check(std::abs(sample.slope[0] - derivative) < 1e-10, "Spatial irregularity derivative mismatch");
        auto invalid = c;
        invalid.rail.foundation_damping = -1;
        Reject([&] { invalid.Validate(); }, "Negative damping accepted");
        BeamChecks();
        VehicleChecks(p);
        CouplingChecks(p, c);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

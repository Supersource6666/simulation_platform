#include "VehicleModel.h"
#include "Track.h"
#include <algorithm>
#include <iomanip>
#include <ostream>
#include "chrono/physics/ChLinkLock.h"

namespace railway {
using namespace chrono;
namespace {
class SuspensionForce : public ChLinkTSDA::ForceFunctor {
  public:
    SuspensionForce(double stiffness, double count, const std::vector<DamperPoint>& table)
        : k(stiffness), count(count), table(table) {}
    double evaluate(double, double rest, double length, double velocity, const ChLinkTSDA&) override {
        return k * (rest - length) - count * DamperForce(table, velocity);
    }
  private:
    double k, count;
    std::vector<DamperPoint> table;
};

class TrackSupportForce : public ChLinkTSDA::ForceFunctor {
  public:
    TrackSupportForce(const Parameters& parameters, double delay) : p(parameters), delay(delay) {}
    double evaluate(double time, double rest, double length, double velocity, const ChLinkTSDA&) override {
        const auto input = TrackInput(p, time, delay);
        return -p.support_k * (length - rest - input.first) - p.support_c * (velocity - input.second);
    }
  private:
    Parameters p;
    double delay;
};
}

std::shared_ptr<ChBody> VehicleModel::AddBody(const std::string& name, double mass, double x, double z) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName(name);
    body->SetMass(mass);
    // Rotation is constrained in this 7-DOF model; these placeholder inertias do not affect vertical dynamics.
    body->SetInertiaXX(ChVector3d(1, 1, 1));
    body->SetPos(ChVector3d(x, 0, z));
    body->EnableCollision(false);
    system.AddBody(body);
    auto guide = chrono_types::make_shared<ChLinkLockPrismatic>();
    guide->Initialize(body, ground, ChFramed(body->GetPos()));  // Free translation along global Z only.
    system.AddLink(guide);
    bodies.push_back(body);
    return body;
}

std::shared_ptr<ChLinkTSDA> VehicleModel::AddSpring(std::shared_ptr<ChBody> lower, std::shared_ptr<ChBody> upper,
                                                  double x, double k, double c, double preload, double lower_z, double upper_z) {
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(lower, upper, false, ChVector3d(x, 0, lower_z), ChVector3d(x, 0, upper_z));
    spring->SetRestLength(upper_z - lower_z + preload / k);
    spring->SetSpringCoefficient(k);
    spring->SetDampingCoefficient(c);
    spring->IsStiff(true);
    system.AddLink(spring);
    springs.push_back(spring);
    return spring;
}

VehicleModel::VehicleModel(const Parameters& parameters) : p(parameters),
    heights{p.car_height, p.bogie_height, p.bogie_height, p.wheel_radius, p.wheel_radius, p.wheel_radius, p.wheel_radius} {
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -p.gravity));
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    system.AddBody(ground);

    const double half_spacing = p.bogie_spacing / 2;
    const double half_wheelbase = p.wheelbase / 2;
    auto car = AddBody("car", p.car_mass, 0, heights[0]);
    auto front = AddBody("bogie_front", p.bogie_mass, half_spacing, heights[1]);
    auto rear = AddBody("bogie_rear", p.bogie_mass, -half_spacing, heights[2]);
    const std::array<double, 4> x = {half_spacing + half_wheelbase, half_spacing - half_wheelbase,
                                   -half_spacing + half_wheelbase, -half_spacing - half_wheelbase};
    for (int i = 0; i < 4; ++i)
        AddBody("wheelset_" + std::to_string(i + 1), p.wheelset_mass, x[i], heights[i + 3]);

    const double secondary_load = p.car_mass * p.gravity / 2;
    const double primary_load = (p.car_mass / 4 + p.bogie_mass / 2) * p.gravity;
    const double axle_load = primary_load + p.wheelset_mass * p.gravity;
    for (const auto& bogie : {front, rear}) {
        auto secondary = AddSpring(bogie, car, bogie->GetPos().x(), p.secondary_k, p.secondary_c, secondary_load,
                                   p.secondary_lower_z, p.secondary_upper_z);
        if (!p.secondary_curve.empty())
            secondary->RegisterForceFunctor(chrono_types::make_shared<SuspensionForce>(
                p.secondary_k, p.secondary_damper_count, p.secondary_curve));
    }
    for (int i = 0; i < 4; ++i) {
        auto primary = AddSpring(bodies[i + 3], i < 2 ? front : rear, x[i], p.primary_k, p.primary_c, primary_load,
                                 p.primary_lower_z, p.primary_upper_z);
        if (!p.primary_curve.empty())
            primary->RegisterForceFunctor(chrono_types::make_shared<SuspensionForce>(
                p.primary_k, p.primary_damper_count, p.primary_curve));
        delays[i] = x[0] - x[i];
        supports[i] = AddSpring(ground, bodies[i + 3], x[i], p.support_k, p.support_c, axle_load, 0, p.wheel_radius);
        supports[i]->RegisterForceFunctor(chrono_types::make_shared<TrackSupportForce>(p, delays[i]));
    }
    system.Update();
}

void VehicleModel::Step(double dt) {
    system.DoStepDynamics(dt);
    system.Update();  // Report forces at the same state/time as the exported positions.
}

std::array<double, 7> VehicleModel::Displacements() const {
    std::array<double, 7> result;
    for (int i = 0; i < 7; ++i)
        result[i] = bodies[i]->GetPos().z() - heights[i];
    return result;
}

double VehicleModel::MinSupportForce() const {
    double force = supports[0]->GetForce();
    for (const auto& support : supports)
        force = std::min(force, support->GetForce());
    return force;
}

double VehicleModel::SupportForceSum() const {
    double force = 0;
    for (const auto& support : supports)
        force += support->GetForce();
    return force;
}

void VehicleModel::WriteHeader(std::ostream& out) const {
    out << "time_s";
    for (const auto& body : bodies)
        out << "," << body->GetName() << "_dz_m," << body->GetName() << "_vz_m_s," << body->GetName() << "_az_m_s2";
    for (int i = 0; i < 4; ++i)
        out << ",track_" << i + 1 << "_z_m,support_" << i + 1 << "_N";
    for (int i = 0; i < 4; ++i)
        out << ",track_" << i+1 << "_s_m,track_" << i+1 << "_Lv_m,track_" << i+1
            << "_Rv_m,track_" << i+1 << "_Ld_m,track_" << i+1 << "_Rd_m,track_" << i+1 << "_measured";
    out << ",secondary_front_N,secondary_rear_N,primary_1_N,primary_2_N,primary_3_N,primary_4_N\n";
}

void VehicleModel::WriteRow(std::ostream& out) const {
    out << std::setprecision(12) << Time();
    const auto displacement = Displacements();
    for (int i = 0; i < 7; ++i)
        out << "," << displacement[i] << "," << bodies[i]->GetPosDt().z() << "," << bodies[i]->GetPosDt2().z();
    for (int i = 0; i < 4; ++i)
        out << "," << TrackInput(p, Time(), delays[i]).first << "," << supports[i]->GetForce();
    for (int i = 0; i < 4; ++i) {
        const auto sample = TrackChannels(p, Time(), delays[i]);
        out << "," << sample.s;
        for (double channel : sample.displacement) out << "," << channel;
        out << "," << sample.measured;
    }
    out << "," << springs[0]->GetForce() << "," << springs[1]->GetForce();
    for (int i = 0; i < 4; ++i)
        out << "," << springs[2 + 2 * i]->GetForce();
    out << "\n";
}
}  // namespace railway

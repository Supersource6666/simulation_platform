#include "VehicleModel3D.h"
#include "Track.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ostream>
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

namespace railway {
using namespace chrono;

namespace {
// Vertical damper characteristic (odd-symmetric, segment-wise linear) shared with the legacy model.
class VerticalDamper : public ChLinkTSDA::ForceFunctor {
  public:
    VerticalDamper(double stiffness, double count, const std::vector<DamperPoint>& table)
        : k(stiffness), count(count), table(table) {}
    double evaluate(double, double rest, double length, double velocity, const ChLinkTSDA&) override {
        // Bump-stop: prevent the spring from being compressed past its solid length. Without this
        // the anchors can cross (length -> 0), flipping the TSDA direction and turning the spring
        // into geometric negative stiffness that pulls the bogie/carbody down and diverges.
        const double solid = std::max(0.02, rest * 0.1);
        double k_eff = k;
        if (length < solid)
            k_eff = k * (1.0 + 2000.0 * (solid - length) / solid);
        return k_eff * (rest - length) - count * DamperForce(table, velocity);
    }
  private:
    double k, count;
    std::vector<DamperPoint> table;
};
// Per-rail vertical support: one-sided linear compression spring with damping, plus the
// prescribed Lv / Rv kinematic displacement. The spring is disengaged (zero force) when the
// wheelset has crossed below the rail, so the bilinear-spring inversion cannot drive the wheel
// further into the ground; the preset compression still supplies the static preload at rest.
class WheelRailSupportForce : public ChLinkTSDA::ForceFunctor {
  public:
    WheelRailSupportForce(const Parameters& parameters, double delay, int side /*-1 left, +1 right*/)
        : p(parameters), delay(delay), side(side) {}
    double evaluate(double time, double rest, double length, double velocity, const ChLinkTSDA& link) override {
        const auto v = TrackVerticalSplit(p, time, delay, p.rolling_gauge, p.support_k);
        const double input = side < 0 ? v.z_left : v.z_right;
        const double input_v = side < 0 ? v.vz_left : v.vz_right;
        // One-sided rail contact: the rail can only push the wheel (compression), never pull it
        // (tension). Once the wheelset lifts off (compression >= 0) the support force must vanish;
        // otherwise the spring enters tension and pulls the wheel downward, producing a large
        // negative support force that falsifies MinSupportForce() and destabilises the bounce mode.
        const double compression = length - rest - input;
        if (compression >= 0)
            return 0.0;
        const double F_damp = -p.support_c * (velocity - input_v);
        return -p.support_k * compression + F_damp;
    }
  private:
    Parameters p;
    double delay;
    int side;
};
// Wheelset lateral wheel-rail contact. Combines:
//   - Lateral creep (Kalker f22) opposing relative lateral velocity (v_y_w - V*psi_w).
//   - Spin creep contribution absorbed in the rotational spring/damper link.
//   - Linear conicity gravitational restoring force, very stiff lateral contact, and explicit
//     track inputs (mean of Ld/Rd) for irregularity excitation.
//   - Cant-induced lateral gravity component (sin(cant) * m_w * g).
//   - Quasi-static centrifugal acceleration V^2 / R for a constant-curvature track.
// The link measures `velocity` along the world Y axis connecting the two anchors.
class LateralCreep : public ChLinkTSDA::ForceFunctor {
  public:
    LateralCreep(const Parameters& parameters, double delay) : p(parameters), delay(delay) {}
    double evaluate(double time, double rest, double length, double velocity, const ChLinkTSDA& link) override {
        const auto in = TrackLateralInput(p, time, delay);
        // ChLink::GetBody2 is the wheelset (link initialised as Initialize(ground, wheelset, ...)),
        // so read its rotation / angular velocity for the creep yaw term.
        const auto* bodyA = static_cast<const ChBody*>(link.GetBody2());
        const ChVector3d angWorld = bodyA->GetRot().Rotate(bodyA->GetAngVelLocal());
        const double psi_w = angWorld.z();
        const double v_y_w = velocity;
        // Kalker linear lateral creep. The creep coefficients are force per unit CREEPAGE
        // (dimensionless relative velocity), so the raw velocity difference must be
        // normalised by the rolling speed V: xi_y = (v_y_w - V * psi_w) / V.
        const double f_creep = -p.creep_lat * (v_y_w - p.speed * psi_w) / p.speed;
        // Conicity gravitational stiffness: gravity component proportional to lateral offset.
        const double f_conicity = -p.wheelset_mass * p.gravity * p.wheel_conicity * (length - rest);
        // Lateral contact stiffness: very stiff bilateral spring to the rail.
        const double f_contact = -p.lateral_contact_k * (length - rest);
        // Track alignment irregularity: (Ld - Rd)/2.
        const double f_alignment = -p.lateral_contact_k * in.alignment;
        // Cant-induced gravity on the wheelset lateral direction.
        const double f_cant = -p.wheelset_mass * p.gravity * std::sin(p.track_cant);
        // Centripetal forcing on a curve of radius R = 1 / curvature (toward curve centre).
        double f_curve = 0;
        if (p.track_curvature > 0) {
            f_curve = -p.wheelset_mass * p.speed * p.speed * p.track_curvature;
        }
        // Track irregularity (mean of Ld/Rd) as kinematic lateral excitation.
        const double f_track = -p.lateral_contact_k * in.y_track;
        return f_creep + f_conicity + f_contact + f_alignment + f_cant + f_curve + f_track;
    }
  private:
    Parameters p;
    double delay;
};
// Spin creep (yaw moment) about vertical axis; opposing body is ground (yaw=0).
class LateralSpin : public ChLinkRSDA::TorqueFunctor {
  public:
    LateralSpin(const Parameters& parameters) : p(parameters) {}
    double evaluate(double, double, double angle, double rate, const ChLinkRSDA&) override {
        // Spin creep damping opposes the yaw rate; creep torque is per unit spin CREEPAGE
        // (rate / V), consistent with the Kalker f33 coefficient units [N*m].
        double m_creep = -p.creep_spin * rate / p.speed;
        // Curve-following moment from conicity (linearised wheelset model): -f11 * lambda * b / R.
        double m_curve = 0;
        if (p.track_curvature > 0) {
            m_curve = -p.creep_long * p.wheel_conicity * (p.rolling_gauge / 2) * p.track_curvature;
        }
        return m_creep + m_curve;
    }
  private:
    Parameters p;
};

inline ChVector3d WorldVec(double x, double y, double z) { return ChVector3d(x, y, z); }
// Compute the quaternion that rotates the unit Z axis to the requested world direction.
inline ChQuaterniond AlignZToAxis(const ChVector3d& axis_world) {
    ChVector3d z(0, 0, 1);
    const double cos_a = Vdot(z, axis_world);
    if (cos_a > 0.999999) return QuatFromAngleAxis(0, ChVector3d(0, 1, 0));
    if (cos_a < -0.999999) return QuatFromAngleAxis(CH_PI, ChVector3d(1, 0, 0));
    ChVector3d k = Vcross(z, axis_world);
    k.Normalize();
    return QuatFromAngleAxis(std::acos(cos_a), k);
}
}  // namespace

std::shared_ptr<ChBody> VehicleModel3D::AddBody(const std::string& name, double mass,
                                                ChVector3d inertia, const ChVector3d& pos) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName(name);
    body->SetMass(mass);
    body->SetInertiaXX(inertia);
    body->SetPos(pos);
    body->EnableCollision(false);
    system.AddBody(body);
    return body;
}

std::shared_ptr<ChLinkTSDA> VehicleModel3D::AddTranslationalSpring(std::shared_ptr<ChBody> lower,
                                                                   std::shared_ptr<ChBody> upper,
                                                                   ChVector3d /*axis*/,
                                                                   const ChVector3d& point_lower,
                                                                   const ChVector3d& point_upper,
                                                                   double k, double c, double preload) {
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    const double anchor_distance = (point_upper - point_lower).Length();
    spring->Initialize(lower, upper, false, point_lower, point_upper);
    spring->SetRestLength(anchor_distance + preload / k);
    spring->SetSpringCoefficient(k);
    spring->SetDampingCoefficient(c);
    spring->IsStiff(true);
    system.AddLink(spring);
    links.push_back(spring);
    return spring;
}

std::shared_ptr<ChLinkRSDA> VehicleModel3D::AddRotationalSpring(std::shared_ptr<ChBody> a,
                                                               std::shared_ptr<ChBody> b,
                                                               ChVector3d axis_world,
                                                               double k, double c) {
    auto rsda = chrono_types::make_shared<ChLinkRSDA>();
    // Local frames on both bodies have their Z axis aligned with the requested world axis.
    // The bodies start at identity orientation, so body-local == world. Once the bodies rotate
    // the local Z travels with them, but since each body has only one rotational DOF tracked
    // by a single RSDA, the cross-coupling from other rotations stays second-order.
    const ChQuaterniond q_align = AlignZToAxis(axis_world);
    ChFrame<> frameA(ChVector3d(0, 0, 0), q_align);
    ChFrame<> frameB(ChVector3d(0, 0, 0), q_align);
    rsda->Initialize(a, b, true, frameA, frameB);
    rsda->SetSpringCoefficient(k);
    rsda->SetDampingCoefficient(c);
    system.AddLink(rsda);
    rsdas.push_back(rsda);
    return rsda;
}

void VehicleModel3D::BuildSuspension() {
    const double half_spacing = p.bogie_spacing / 2;
    const double half_wheelbase = p.wheelbase / 2;
    const double y_primary = p.primary_y_offset;
    const double y_secondary = p.secondary_y_offset;
    const double primary_per_side_load = (p.car_mass / 4 + p.bogie_mass / 2) * p.gravity / 2;
    const double secondary_per_side_load = (p.car_mass / 2) * p.gravity / 2;

    auto build_primary = [&](int w_index, std::shared_ptr<ChBody> bogie, double x_world) {
        // For each wheelset, build symmetric springs on BOTH sides so vertical forces do not
        // generate a net roll moment about the wheelset centre. The damper count is split too.
        for (int side : {-1, +1}) {
            const double y_anchor = side * y_primary;
            // 1. Vertical primary (Z axis) on this side.
            auto v_spring = AddTranslationalSpring(
                wheelsets[w_index], bogie, WorldVec(0, 0, 1),
                WorldVec(x_world, y_anchor, p.primary_lower_z),
                WorldVec(x_world, y_anchor, p.primary_upper_z),
                p.primary_k / 2, p.primary_c, primary_per_side_load);
            if (!p.primary_curve.empty())
                v_spring->RegisterForceFunctor(chrono_types::make_shared<VerticalDamper>(
                    p.primary_k / 2, p.primary_damper_count / 2.0, p.primary_curve));
            // 2. Lateral primary suspension is modelled by the wheel-rail lateral creep / contact
                    // stiffness and the yaw restraint; a separate lateral TSDA here would double-count
                    // the lateral channel and, with a vertical anchor span, act as an unintended
                    // vertical spring (historical bug), so it is intentionally omitted.
        }
        // 3. Wheelset roll (pitch about its own Y axis). The wheelset has no longitudinal DOF so
        // this roll does not accumulate and has no effect on the contact point (which lies on the
        // roll axis); lateral effects are captured by the conicity + lateral-creep force functor.
        // A stiff "pitch lock to ground" here couples into the bogie bounce mode and drives the
        // bogie/carbody into the geometric negative-stiffness region of the primary springs
        // (anchor crossover), producing the vertical divergence, so keep only a modest restraint
        // that damps the free roll angle without locking it.
        const double pitch_k = p.primary_pitch_k > 0 ? p.primary_pitch_k : 1.0e2;
        const double pitch_c = p.primary_pitch_c > 0 ? p.primary_pitch_c : 1.0e2;
        AddRotationalSpring(wheelsets[w_index], ground, WorldVec(0, 1, 0), pitch_k, pitch_c);
        // 4. Yaw RSDA (allows hunting motion).
        AddRotationalSpring(wheelsets[w_index], bogie, WorldVec(0, 0, 1),
                            p.yaw_primary_k, p.yaw_primary_c);
        // 5. Roll RSDA between wheelset and bogie (firm).
        AddRotationalSpring(wheelsets[w_index], bogie, WorldVec(1, 0, 0),
                            p.roll_primary_k, p.roll_primary_c);
    };
    auto build_secondary = [&](std::shared_ptr<ChBody> bogie, double x_world) {
        for (int side : {-1, +1}) {
            const double y_anchor = side * y_secondary;
            // 1. Vertical secondary between bogie and carbody on this side.
            auto v_spring = AddTranslationalSpring(
                bogie, car, WorldVec(0, 0, 1),
                WorldVec(x_world, y_anchor, p.secondary_lower_z),
                WorldVec(x_world, y_anchor, p.secondary_upper_z),
                p.secondary_k / 2, p.secondary_c, secondary_per_side_load);
            if (!p.secondary_curve.empty())
                v_spring->RegisterForceFunctor(chrono_types::make_shared<VerticalDamper>(
                    p.secondary_k / 2, p.secondary_damper_count / 2.0, p.secondary_curve));
            // 2. Lateral secondary suspension is carried by the secondary yaw restraint and the
                    // wheel-rail lateral creep; a separate lateral TSDA is omitted to avoid the
                    // historical vertical-spring bug and double-counting the lateral channel.
        }
        // 3. Pitch RSDA (anti-pitch) carbody<->bogie.
        AddRotationalSpring(bogie, car, WorldVec(0, 1, 0),
                            p.secondary_pitch_k, p.secondary_pitch_c);
        // 4. Yaw RSDA (anti-hunting damper) carbody<->bogie.
        AddRotationalSpring(bogie, car, WorldVec(0, 0, 1),
                            p.yaw_secondary_k, p.yaw_secondary_c);
        // 5. Roll RSDA (anti-roll bar) carbody<->bogie.
        AddRotationalSpring(bogie, car, WorldVec(1, 0, 0),
                            p.roll_secondary_k, p.roll_secondary_c);
    };
    const std::array<double, 4> wheel_x = {
        half_spacing + half_wheelbase, half_spacing - half_wheelbase,
        -half_spacing + half_wheelbase, -half_spacing - half_wheelbase};
    for (int i = 0; i < 4; ++i) {
        build_primary(i, i < 2 ? bogies[0] : bogies[1], wheel_x[i]);
        axle_x_world[i] = wheel_x[i];
        axle_axial_offset[i] = wheel_x[0] - wheel_x[i];
    }
    build_secondary(bogies[0], half_spacing);
    build_secondary(bogies[1], -half_spacing);
}

void VehicleModel3D::BuildWheelRailContact() {
    const double half_gauge = p.rolling_gauge / 2;
    for (int i = 0; i < 4; ++i) {
        const double x = axle_x_world[i];
        const double delay = axle_axial_offset[i];
        const double static_axle_load = (p.car_mass / 4 + p.bogie_mass / 2 + p.wheelset_mass) * p.gravity;
        const double preload_per_rail = static_axle_load / 2;
        auto left = AddTranslationalSpring(
            ground, wheelsets[i], WorldVec(0, 0, 1),
            WorldVec(x, -half_gauge, 0), WorldVec(x, -half_gauge, p.wheel_radius),
            p.support_k * p.support_lr_balance, p.support_c, preload_per_rail);
        auto right = AddTranslationalSpring(
            ground, wheelsets[i], WorldVec(0, 0, 1),
            WorldVec(x, +half_gauge, 0), WorldVec(x, +half_gauge, p.wheel_radius),
            p.support_k * (2 - p.support_lr_balance), p.support_c, preload_per_rail);
        // The WheelRailSupportForce functor provides the prescribed vertical displacement (Lv / Rv)
        // and the rail-velocity coupling through the existing built-in stiffness/damping.
        left->RegisterForceFunctor(chrono_types::make_shared<WheelRailSupportForce>(p, delay, -1));
        right->RegisterForceFunctor(chrono_types::make_shared<WheelRailSupportForce>(p, delay, +1));
        // Lateral wheel-rail contact (Y direction). Anchors coincide in world space at
        // (x, 0, wheel_radius) so the wheelset sits at the rest position (length=0) on a tangent
        // contact with the rail centre-line. The 1 mm anchor offset and 1 mm rest length used
        // previously produced a bistable spring with a spurious secondary equilibrium at y=-2 mm
        // (force direction flips but force magnitude is non-zero when anchors cross).
        auto lateral = chrono_types::make_shared<ChLinkTSDA>();
        lateral->Initialize(ground, wheelsets[i], false,
                            WorldVec(x, 0, p.wheel_radius),
                            WorldVec(x, 0, p.wheel_radius));
        lateral->SetSpringCoefficient(p.lateral_contact_k);
        lateral->SetDampingCoefficient(0);
        lateral->SetRestLength(0);
        lateral->IsStiff(true);
        lateral->RegisterForceFunctor(chrono_types::make_shared<LateralCreep>(p, delay));
        system.AddLink(lateral);
        links.push_back(lateral);
        // Spin creep (yaw moment) about world Z, with custom torque functor.
        auto spin = chrono_types::make_shared<ChLinkRSDA>();
        const ChQuaterniond q_align = AlignZToAxis(ChVector3d(0, 0, 1));
        ChFrame<> frame(ChVector3d(x, 0, p.wheel_radius), q_align);
        spin->Initialize(ground, wheelsets[i], true, frame, frame);
        spin->SetSpringCoefficient(0);
        spin->SetDampingCoefficient(0);
        spin->RegisterTorqueFunctor(chrono_types::make_shared<LateralSpin>(p));
        system.AddLink(spin);
        rsdas.push_back(spin);
        (void)static_axle_load;
    }
}

VehicleModel3D::VehicleModel3D(const Parameters& parameters) : p(parameters) {
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -p.gravity));
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    // HHT integrator with strong numerical damping (alpha = -1/3 maximum) gives the stiff
    // wheel-rail bilateral support enough dissipation to remain stable; the legacy Newmark
    // integrator accumulates a small per-step drift on the rotational RSDA pivots over the long
    // self-test horizon because the linearised Jacobian is recomputed at the perturbed state
    // every iteration.
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto* hht = static_cast<ChTimestepperHHT*>(system.GetTimestepper().get()))
        hht->SetAlpha(-1.0 / 3.0);
    ground = chrono_types::make_shared<ChBody>();
    ground->SetName("ground");
    ground->SetFixed(true);
    ground->EnableCollision(false);
    system.AddBody(ground);

    car = AddBody("car", p.car_mass,
                  ChVector3d(p.car_inertia.Ixx, p.car_inertia.Iyy, p.car_inertia.Izz),
                  WorldVec(0, 0, p.car_height));
    bogies[0] = AddBody("bogie_front", p.bogie_mass,
                        ChVector3d(p.bogie_inertia.Ixx, p.bogie_inertia.Iyy, p.bogie_inertia.Izz),
                        WorldVec(p.bogie_spacing / 2, 0, p.bogie_height));
    bogies[1] = AddBody("bogie_rear", p.bogie_mass,
                        ChVector3d(p.bogie_inertia.Ixx, p.bogie_inertia.Iyy, p.bogie_inertia.Izz),
                        WorldVec(-p.bogie_spacing / 2, 0, p.bogie_height));
    const double half_wheelbase = p.wheelbase / 2;
    const double half_spacing = p.bogie_spacing / 2;
    const std::array<double, 4> x = {
        half_spacing + half_wheelbase, half_spacing - half_wheelbase,
        -half_spacing + half_wheelbase, -half_spacing - half_wheelbase};
    for (int i = 0; i < 4; ++i)
        wheelsets[i] = AddBody("wheelset_" + std::to_string(i + 1), p.wheelset_mass,
                               ChVector3d(p.wheelset_inertia.Ixx, p.wheelset_inertia.Iyy,
                                          p.wheelset_inertia.Izz),
                               WorldVec(x[i], 0, p.wheel_radius));

    BuildSuspension();
    BuildWheelRailContact();
    system.Update();
}

void VehicleModel3D::Step(double dt) {
    system.DoStepDynamics(dt);
    system.Update();
}

namespace {
// Quaternion -> Euler ZYX (roll, pitch, yaw) extraction. ChQuaterniond uses (e0,e1,e2,e3) for (w,x,y,z).
inline void QuatToEulerZYX(const ChQuaterniond& rot, double& roll, double& pitch, double& yaw) {
    const double w = rot.e0(), x = rot.e1(), y = rot.e2(), z = rot.e3();
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (w * y - z * x);
    pitch = std::abs(sinp) >= 1 ? std::copysign(CH_PI / 2, sinp) : std::asin(sinp);
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

VehicleModel3D::BodyState VehicleModel3D::Car() const {
    BodyState s{};
    s.z = car->GetPos().z() - p.car_height;
    s.y = car->GetPos().y();
    QuatToEulerZYX(car->GetRot(), s.roll, s.pitch, s.yaw);
    return s;
}
VehicleModel3D::BodyState VehicleModel3D::Bogie(int i) const {
    BodyState s{};
    const auto b = bogies[i];
    s.z = b->GetPos().z() - p.bogie_height;
    s.y = b->GetPos().y();
    QuatToEulerZYX(b->GetRot(), s.roll, s.pitch, s.yaw);
    return s;
}
VehicleModel3D::BodyState VehicleModel3D::Wheelset(int i) const {
    BodyState s{};
    const auto w = wheelsets[i];
    s.z = w->GetPos().z() - p.wheel_radius;
    s.y = w->GetPos().y();
    QuatToEulerZYX(w->GetRot(), s.roll, s.pitch, s.yaw);
    return s;
}

double VehicleModel3D::AxleSupport(int i) const {
    double sum = 0;
    for (const auto& link : links) {
        const auto* tsda = dynamic_cast<const ChLinkTSDA*>(link.get());
        if (!tsda) continue;
        // Body 2 must be the wheelset and the spring direction must be along world Z. This
        // excludes the lateral creep, primary lateral, secondary lateral springs (all along Y).
        if (tsda->GetBody2() != wheelsets[i].get()) continue;
        const auto dir = (tsda->GetPoint1Abs() - tsda->GetPoint2Abs()).GetNormalized();
        // Use a tight Z-direction threshold. The wheel-rail support springs have anchors at
        // identical (x, y) and differ only in z, so their direction stays aligned with world Z
        // even under large bounce amplitudes. The lateral primary/secondary springs, however,
        // have anchors at different z (primary_lower_z vs. primary_upper_z) so when the wheelset
        // drops its anchor drops too and the spring tilts enough that |dir.z| drops below 1.
        // Counting those tilted lateral springs as wheel-rail support double-counts the bounce
        // response and produces the spurious negative support force when the lateral creep
        // spring is in tension. A 0.999 cutoff keeps the support springs in (they stay > 0.9999
        // even at 10 cm bounce) while excluding the tilted lateral springs.
        if (std::abs(dir.z()) > 0.999)
            sum += tsda->GetForce();
    }
    return sum;
}
double VehicleModel3D::SupportForceSum() const {
    double s = 0;
    for (int i = 0; i < 4; ++i) s += AxleSupport(i);
    return s;
}
double VehicleModel3D::MinSupportForce() const {
    double m = AxleSupport(0);
    for (int i = 1; i < 4; ++i) m = std::min(m, AxleSupport(i));
    return m;
}

void VehicleModel3D::WriteHeader(std::ostream& out) const {
    out << "time_s";
    const std::array<const char*, 7> body_names = {"car", "bogie_front", "bogie_rear",
                                                    "wheelset_1", "wheelset_2", "wheelset_3", "wheelset_4"};
    const std::array<const char*, 5> state_names = {"z_m", "pitch_rad", "y_m", "roll_rad", "yaw_rad"};
    const std::array<const char*, 5> rate_names = {"vz_m_s", "vpitch_rad_s", "vy_m_s", "vroll_rad_s", "vyaw_rad_s"};
    for (const auto* name : body_names) {
        for (int s = 0; s < 5; ++s) out << "," << name << "_" << state_names[s];
        for (int s = 0; s < 5; ++s) out << "," << name << "_" << rate_names[s];
    }
    for (int i = 0; i < 4; ++i)
        out << ",track_" << i + 1 << "_Lv,track_" << i + 1 << "_Rv,track_" << i + 1
            << "_Ld,track_" << i + 1 << "_Rd,support_" << i + 1 << "_N";
    out << "\n";
}
void VehicleModel3D::WriteRow(std::ostream& out) const {
    out << std::setprecision(12) << Time();
    auto write_body = [&](const std::shared_ptr<ChBody>& body) {
        const auto pos = body->GetPos();
        const auto rot = body->GetRot();
        double roll = 0, pitch = 0, yaw = 0;
        QuatToEulerZYX(rot, roll, pitch, yaw);
        out << "," << pos.z() << "," << pitch << "," << pos.y() << "," << roll << "," << yaw;
        const auto linVel = body->GetPosDt();
        const auto angVelWorld = rot.Rotate(body->GetAngVelLocal());
        out << "," << linVel.z() << "," << angVelWorld.y() << "," << linVel.y()
            << "," << angVelWorld.x() << "," << angVelWorld.z();
    };
    write_body(car);
    for (int i = 0; i < 2; ++i) write_body(bogies[i]);
    for (int i = 0; i < 4; ++i) write_body(wheelsets[i]);
    for (int i = 0; i < 4; ++i) {
        const auto sample = TrackChannels(p, Time(), axle_axial_offset[i]);
        out << "," << sample.displacement[0] << "," << sample.displacement[1]
            << "," << sample.displacement[2] << "," << sample.displacement[3]
            << "," << AxleSupport(i);
    }
    out << "\n";
}
}  // namespace railway
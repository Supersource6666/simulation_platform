#pragma once
#include "Parameters.h"
#include <array>
#include <memory>
#include <string>
#include <vector>
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChSystemNSC.h"

namespace railway {
// Spatial vehicle model that extends the vertical 7-DOF reduction with nodding (pitch), lateral
// translation and yaw (anti-hunting). Degrees of freedom:
//   - Carbody:   {z, pitch_y, y, roll_x, yaw_z}                = 5 DOF
//   - 2 Bogies:  {z, pitch_y, y, roll_x, yaw_z}    x 2        = 10 DOF
//   - 4 Wheelsets:{z, y, yaw_z}                       x 4       = 12 DOF
// Total: 27 DOF. Wheelset roll and longitudinal translation stay constrained (slender wheelset
// approximation, consistent with documented 25T parameters).
// The lateral/hunting wheel-rail contact is implemented through three orthogonal channels:
//   - vertical (left/right rails, separate bilateral support)
//   - lateral  (Kalker f22 creep * (v_y - V*psi_w) + conicity gravitational stiffness)
//   - spin     (Kalker f33 creep * psi_dot)
// The longitudinal channel is collapsed: pure rolling without wheel rotation accumulation because
// the wheelset body retains no longitudinal DOF; forward speed V enters the creep law as a parameter.
class VehicleModel3D {
  public:
    explicit VehicleModel3D(const Parameters& parameters);
    void Step(double dt);
    void WriteHeader(std::ostream& out) const;
    void WriteRow(std::ostream& out) const;

    // Per-body summary observations (z, pitch, y, roll, yaw) in world-aligned frame.
    struct BodyState { double z, pitch, y, roll, yaw; };
    BodyState Car() const;
    BodyState Bogie(int index) const;          // 0 == front, 1 == rear.
    BodyState Wheelset(int index) const;       // 0..3, ordered front-outboard, front-inboard, rear-inboard, rear-outboard.
    // Per-axle vertical support force summed over the two rails of one wheelset [N].
    double AxleSupport(int index) const;
    // Aggregate over the four wheelsets.
    double SupportForceSum() const;
    double MinSupportForce() const;
    // Time helpers.
    double Time() const { return system.GetChTime(); }

  private:
    // Body factory for free 6-DOF ChBody.
    std::shared_ptr<chrono::ChBody> AddBody(const std::string& name, double mass,
                                            chrono::ChVector3d inertia,
                                            const chrono::ChVector3d& pos);
    // Translational spring between two bodies along world (X, Y, Z); axis is unit direction vector.
    std::shared_ptr<chrono::ChLinkTSDA> AddTranslationalSpring(std::shared_ptr<chrono::ChBody> lower,
                                                               std::shared_ptr<chrono::ChBody> upper,
                                                               chrono::ChVector3d axis,
                                                               const chrono::ChVector3d& point_lower,
                                                               const chrono::ChVector3d& point_upper,
                                                               double k, double c, double preload);
    // Rotational spring whose frame Z axis equals `axis_world` (in body 1 frame) and Z axis equals
    // `axis_world` (in body 2 frame) for symmetric joint geometry. Rest angle is auto from init.
    std::shared_ptr<chrono::ChLinkRSDA> AddRotationalSpring(std::shared_ptr<chrono::ChBody> a,
                                                           std::shared_ptr<chrono::ChBody> b,
                                                           chrono::ChVector3d axis_world,
                                                           double k, double c);
    // Build full vertical+pitch coupling (primary, secondary, support) plus lateral+yaw+roll links.
    void BuildSuspension();
    // Build wheel-rail contact (left/right vertical supports + lateral+yaw creep links).
    void BuildWheelRailContact();

    Parameters p;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::ChBody> ground;
    // Bodies ordered: carbody, bogie_front, bogie_rear, wheelset_1..4.
    std::shared_ptr<chrono::ChBody> car, bogies[2], wheelsets[4];
    std::vector<std::shared_ptr<chrono::ChLinkTSDA>> links;
    std::vector<std::shared_ptr<chrono::ChLinkRSDA>> rsdas;
    // Cache of axle spatial positions in world frame (x) and lateral offsets (y) for delay tracking.
    std::array<double, 4> axle_x_world{};
    std::array<double, 4> axle_axial_offset{};
};
}  // namespace railway
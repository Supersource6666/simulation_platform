#pragma once
#include "Vehicle10DOF.h"
#include <string>

namespace railway::coupled {
enum class ContactState { Contact, Separation };
struct ContactPoint {
    int wheel_id = 0, rail_id = 0;
    double x = 0, gap = 0, penetration = 0, force = 0;
    ContactState state = ContactState::Separation;
};
struct ContactEvent {
    double time;
    int wheel_id, rail_id;
    ContactState from, to;
};
struct HertzContact {
    double coefficient;
    double Force(double compression) const;
    double Tangent(double compression) const;
};
struct SpeedPoint {
    double time, speed;
};
struct Motion {
    double position, speed, acceleration;
};
class SpeedProfile {
  public:
    std::vector<SpeedPoint> points;
    void Validate() const;
    Motion Evaluate(double time, double initial_position) const;
};
struct SimulationConfig {
    RailParameters rail;
    int rail_count = 1;
    bool fem = false;
    double car_pitch_inertia = 1.4067e6;  // scenario assumptions, not source-axis inertias
    double bogie_pitch_inertia = 1216.8;
    double hertz_coefficient = 1e11;  // N/m^(3/2), per contact; axle-equivalent for one rail
    double initial_position = 30;
    double dt = 0.0001, duration = 2;
    int max_iterations = 10;
    double force_tolerance = 1e-7, displacement_tolerance = 1e-9;
    double force_reference = 1e5;
    double max_phase_increment = 0.2;  // conservative internal substep frequency resolution
    SpeedProfile speed;
    std::string vehicle_config;
    static SimulationConfig Load(const std::string& file);
    void Validate() const;
};

class CoupledSimulation {
  public:
    CoupledSimulation(const Parameters& vehicle, const SimulationConfig& config);
    void InitializeStaticEquilibrium();
    void Step(double dt);
    double Time() const { return time; }
    const State& VehicleState() const { return vehicle_state; }
    const std::vector<State>& RailStates() const { return rail_states; }
    const std::vector<ContactPoint>& Contacts() const { return contacts; }
    const std::vector<ContactEvent>& Events() const { return events; }
    TrackResponse RailResponse(int rail, double x) const { return rails.at(rail)->Evaluate(x, rail_states.at(rail)); }
    int Iterations() const { return iterations; }
    double ForceResidual() const { return residual; }
    double StaticResidual() const { return static_residual; }
    int InternalSteps() const { return internal_steps; }
    double MinInternalDt() const { return min_internal_dt; }

  private:
    void Advance(double dt);
    double StableStepEstimate() const;
    Matrix ContactMap(double at_time, Vector& irregularity, std::vector<double>& positions) const;
    void UpdateContacts(const Vector& force, const Vector& compression, const std::vector<double>& positions, bool record);
    Parameters parameters;
    SimulationConfig config;
    Vehicle10DOF vehicle;
    HertzContact hertz;
    std::vector<std::unique_ptr<ITrackDynamics>> rails;
    State vehicle_state;
    std::vector<State> rail_states;
    std::vector<ContactPoint> contacts;
    std::vector<ContactEvent> events;
    Vector forces;
    double time = 0, residual = 0, static_residual = 0;
    int iterations = 0, rail_size = 0, internal_steps = 0;
    double min_internal_dt = 0;
};
}  // namespace railway::coupled
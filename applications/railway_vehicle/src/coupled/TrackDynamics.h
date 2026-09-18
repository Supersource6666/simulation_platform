#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace railway::coupled {
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
constexpr double pi = 3.14159265358979323846;

struct State {
    Vector q, v, a;
    explicit State(int size = 0) : q(Vector::Zero(size)), v(q), a(q) {}
};

struct TrackResponse {
    Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct RailParameters {
    double length = 120;                // m, simply supported at both ends
    double mass_per_length = 60;        // kg/m, ONE rail
    double bending_stiffness = 6.4e6;   // N m^2
    double foundation_stiffness = 6e7;  // N/m^2 (distributed, not one discrete pad)
    double foundation_damping = 3e4;    // N s/m^2
    int modes = 80;
    int elements = 120;
    void Validate() const;
};

// Coordinates are internal to the track backend. Loads are projected with Basis(x).
// Trial operations are pure: coupling iterations always restart from the committed state.
class ITrackDynamics {
  public:
    virtual ~ITrackDynamics() = default;
    virtual Vector Basis(double x) const = 0;
    virtual Vector SlopeBasis(double x) const = 0;
    virtual const Vector& Mass() const = 0;
    virtual const Vector& Stiffness() const = 0;
    virtual const Vector& Damping() const = 0;
    virtual State Predict(const State& initial, const Vector& load_start, const Vector& load_end, double dt) = 0;
    TrackResponse Evaluate(double x, const State& state) const;
    State StaticState(const Vector& load) const;
};

// Exact oscillator transition with linearly interpolated generalized force (FOH).
// Cache only depends on dt and material properties, never on travel speed.
class ModalRail : public ITrackDynamics {
  public:
    explicit ModalRail(const RailParameters& parameters, bool fem = false);
    Vector Basis(double x) const override;
    Vector SlopeBasis(double x) const override;
    const Vector& Mass() const override { return mass; }
    const Vector& Stiffness() const override { return stiffness; }
    const Vector& Damping() const override { return damping; }
    State Predict(const State& initial, const Vector& load_start, const Vector& load_end, double dt) override;

  private:
    Vector Shape(double x, bool derivative) const;
    void BuildFiniteElements();
    RailParameters p;
    bool use_fem;
    Vector mass, stiffness, damping;
    Matrix eigenvectors;
    double cached_dt = -1;
    std::vector<Eigen::Matrix4d> transitions;
};
}  // namespace railway::coupled

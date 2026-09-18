#pragma once
#include "Parameters.h"
#include <array>
#include <memory>
#include <string>
#include <vector>
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemNSC.h"

namespace railway {
class VehicleModel {
  public:
    explicit VehicleModel(const Parameters& parameters);
    void Step(double dt);
    void WriteHeader(std::ostream& out) const;
    void WriteRow(std::ostream& out) const;
    std::array<double, 7> Displacements() const;
    double MinSupportForce() const;
    double SupportForceSum() const;
    double Time() const { return system.GetChTime(); }

  private:
    std::shared_ptr<chrono::ChBody> AddBody(const std::string& name, double mass, double x, double z);
    std::shared_ptr<chrono::ChLinkTSDA> AddSpring(std::shared_ptr<chrono::ChBody> lower,
                                                std::shared_ptr<chrono::ChBody> upper,
                                                double x, double k, double c, double preload, double lower_z, double upper_z);
    Parameters p;
    chrono::ChSystemNSC system;
    std::shared_ptr<chrono::ChBody> ground;
    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    std::vector<std::shared_ptr<chrono::ChLinkTSDA>> springs;
    std::array<std::shared_ptr<chrono::ChLinkTSDA>, 4> supports;
    std::array<double, 4> delays;
    const std::array<double, 7> heights;
};
}  // namespace railway

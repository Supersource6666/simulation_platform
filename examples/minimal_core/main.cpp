#include <cmath>
#include <iomanip>
#include <iostream>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"

int main() {
    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0, -9.81, 0));
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetMass(1.0);
    body->SetPos(chrono::ChVector3d(0, 0, 0));
    body->EnableCollision(false);
    system.AddBody(body);

    constexpr double dt = 0.001;
    constexpr int steps = 1000;
    for (int i = 0; i < steps; ++i) {
        system.DoStepDynamics(dt);
    }

    const double time = system.GetChTime();
    const double y = body->GetPos().y();
    const double vy = body->GetPosDt().y();
    std::cout << std::fixed << std::setprecision(6)
              << "time=" << time << " s, y=" << y << " m, vy=" << vy << " m/s\n";
    // Semi-implicit Euler has a position error of g * time * dt / 2 for constant gravity.
    const bool passed = std::isfinite(time) && std::isfinite(y) && std::isfinite(vy) &&
                        std::abs(time - 1.0) < 1e-10 && std::abs(vy + 9.81) < 1e-8 &&
                        std::abs(y + 4.905) < 0.005;
    std::cout << (passed ? "PASS" : "FAIL") << '\n';
    return passed ? 0 : 1;
}

#pragma once
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace railway {
struct TrackSample {
    double s = 0;
    std::array<double, 4> displacement{};  // Lv, Rv, Ld, Rd [m].
    std::array<double, 4> slope{};         // Derivative with respect to distance.
    bool measured = false;
};
class TrackData {
  public:
    static std::shared_ptr<const TrackData> Load(const std::string& path);
    TrackSample Sample(double s) const;
    double First() const { return position.front(); }
    double Last() const { return position.back(); }
    size_t Size() const { return position.size(); }
    double VerticalScale() const;
  private:
    std::vector<double> position;
    std::vector<std::array<double, 4>> value, derivative;
};
}

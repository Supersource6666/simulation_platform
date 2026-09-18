#include "TrackData.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace railway {
std::shared_ptr<const TrackData> TrackData::Load(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open track file: " + path);
    std::string line;
    std::getline(file, line);
    if (line.size() >= 3 && line.substr(0, 3) == "\xef\xbb\xbf") line.erase(0, 3);
    std::istringstream header(line);
    for (const auto* key : {"s", "Lv", "Rv", "Ld", "Rd"}) {
        std::string actual;
        if (!(header >> actual) || actual != key) throw std::runtime_error("Track header must be: s Lv Rv Ld Rd");
    }
    std::string extra;
    if (header >> extra) throw std::runtime_error("Track file must have exactly five columns");
    auto data = std::make_shared<TrackData>();
    size_t line_number = 1;
    while (std::getline(file, line)) {
        ++line_number;
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream row(line);
        double s;
        std::array<double, 4> values;
        bool valid = bool(row >> s);
        for (auto& v : values) valid = valid && bool(row >> v);
        if (!valid || row >> extra) throw std::runtime_error("Invalid track row " + std::to_string(line_number));
        if (!std::isfinite(s) || (!data->position.empty() && s <= data->position.back()))
            throw std::runtime_error("Track distance must be finite and strictly increasing");
        for (auto v : values)
            if (!std::isfinite(v)) throw std::runtime_error("Nonfinite track displacement");
        data->position.push_back(s);
        data->value.push_back(values);
    }
    if (file.bad()) throw std::runtime_error("Error reading track file");
    const size_t n = data->Size();
    if (n < 2) throw std::runtime_error("Track requires at least two rows");
    data->derivative.resize(n);
    std::vector<double> h(n - 1), d(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) h[i] = data->position[i + 1] - data->position[i];
    auto endpoint = [](double h0, double h1, double d0, double d1) {
        double m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
        if (m * d0 <= 0) return 0.0;
        if (d0 * d1 < 0 && std::abs(m) > 3 * std::abs(d0)) return 3 * d0;
        return m;
    };
    for (int channel = 0; channel < 4; ++channel) {
        for (size_t i = 0; i + 1 < n; ++i)
            d[i] = (data->value[i + 1][channel] - data->value[i][channel]) / h[i];
        if (n == 2) {
            data->derivative[0][channel] = data->derivative[1][channel] = d[0];
            continue;
        }
        data->derivative[0][channel] = endpoint(h[0], h[1], d[0], d[1]);
        data->derivative[n - 1][channel] = endpoint(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
        for (size_t i = 1; i + 1 < n; ++i) {
            double m = 0;
            if (d[i - 1] * d[i] > 0) {
                const double w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1];
                m = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
            }
            data->derivative[i][channel] = m;
        }
    }
    return data;
}

TrackSample TrackData::Sample(double s) const {
    if (!std::isfinite(s) || s < First() - 1e-9 || s > Last() + 1e-9)
        throw std::runtime_error("Requested distance outside measured track");
    s = std::clamp(s, First(), Last());
    const auto upper = std::upper_bound(position.begin(), position.end(), s);
    const size_t i = std::min(static_cast<size_t>(upper - position.begin() - 1), Size() - 2);
    const double h = position[i + 1] - position[i], u = (s - position[i]) / h;
    TrackSample sample;
    sample.s = s;
    sample.measured = true;
    for (int j = 0; j < 4; ++j) {
        const double a = value[i][j], b = value[i + 1][j];
        const double ma = derivative[i][j], mb = derivative[i + 1][j];
        sample.displacement[j] = (2*u*u*u-3*u*u+1)*a + (u*u*u-2*u*u+u)*h*ma +
                                 (-2*u*u*u+3*u*u)*b + (u*u*u-u*u)*h*mb;
        sample.slope[j] = ((6*u*u-6*u)*a + (-6*u*u+6*u)*b)/h +
                          (3*u*u-4*u+1)*ma + (3*u*u-2*u)*mb;
    }
    return sample;
}
double TrackData::VerticalScale() const {
    double scale = 0;
    for (const auto& row : value) scale = std::max(scale, std::abs((row[0] + row[1]) / 2));
    return scale;
}
}

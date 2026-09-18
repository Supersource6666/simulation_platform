#include "Track.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

void Check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
    try {
        Check(argc == 2, "Need measured input path");
        const auto data = railway::TrackData::Load(argv[1]);
        Check(data->Size() == 53348, "Unexpected source row count");
        Check(data->First() == 0 && data->Last() == 13336.75, "Unexpected distance range");
        std::ifstream source(argv[1]);
        std::string line;
        std::getline(source, line);
        size_t rows = 0;
        double previous_s = 0;
        std::array<double, 4> previous{};
        while (std::getline(source, line)) {
            if (line.empty()) continue;
            std::istringstream row(line);
            double s;
            std::array<double, 4> values;
            row >> s;
            for (auto& value : values) row >> value;
            const auto node = data->Sample(s);
            for (int j = 0; j < 4; ++j)
                Check(std::abs(node.displacement[j] - values[j]) < 1e-12, "Interpolation changed an original sample");
            if (rows > 0) {
                const double middle = (previous_s + s) / 2;
                const auto mid = data->Sample(middle);
                const auto left = data->Sample(middle - 1e-5);
                const auto right = data->Sample(middle + 1e-5);
                for (int j = 0; j < 4; ++j) {
                    Check(mid.displacement[j] >= std::min(previous[j], values[j]) - 1e-12 &&
                          mid.displacement[j] <= std::max(previous[j], values[j]) + 1e-12, "Interpolation overshoot");
                    Check(std::abs((right.displacement[j]-left.displacement[j])/2e-5 - mid.slope[j]) < 1e-7,
                          "Incorrect analytic distance derivative");
                }
            }
            previous_s = s;
            previous = values;
            ++rows;
        }
        railway::Parameters p{};
        p.track_data = data; p.track_start = 40; p.track_lead_in = 5; p.speed = 120 / 3.6;
        const double time = (40 + 123.125) / p.speed;
        const auto channels = data->Sample(123.125);
        const auto input = railway::TrackInput(p, time, 0);
        Check(std::abs(input.first - (channels.displacement[0]+channels.displacement[1])/2) < 1e-12, "Wrong vertical channel reduction");
        for (double delay : {0., 2.6, 18., 20.6}) {
            const auto axle = railway::TrackChannels(p, time + delay / p.speed, delay);
            for (int j=0;j<4;++j)
                Check(std::abs(axle.displacement[j]-channels.displacement[j]) < 1e-11, "Incorrect axle delay");
        }
        Check(railway::TrackInput(p, 0, 0).first == 0, "Initial excitation not zero");
        const auto start_left = railway::TrackChannels(p, (40-1e-7)/p.speed, 0);
        const auto start_right = railway::TrackChannels(p, (40+1e-7)/p.speed, 0);
        for (int j=0;j<4;++j) {
            Check(std::abs(start_left.displacement[j]-start_right.displacement[j]) < 1e-8, "Lead-in value discontinuity");
            Check(std::abs(start_left.slope[j]-start_right.slope[j]) < 1e-7, "Lead-in slope discontinuity");
        }
        bool rejected = false;
        try { data->Sample(data->Last()+1); } catch (const std::runtime_error&) { rejected = true; }
        Check(rejected, "Out-of-range interpolation should fail");
        for (const auto* bad : {"s Lv Rv Ld Rd\n0 0 0 0 0\n0 1 1 1 1\n",
                                "s Lv Rv Ld Rd\n0 0 0 0\n1 1 1 1 1\n",
                                "s Rv Lv Ld Rd\n0 0 0 0 0\n1 1 1 1 1\n"}) {
            { std::ofstream fixture("invalid_track_fixture.txt"); fixture << bad; }
            rejected = false;
            try { railway::TrackData::Load("invalid_track_fixture.txt"); } catch (const std::runtime_error&) { rejected = true; }
            Check(rejected, "Invalid input accepted");
        }
        { std::ofstream fixture("linear_track_fixture.txt"); fixture << "s Lv Rv Ld Rd\n10 1 2 3 4\n12 3 4 5 6\n"; }
        const auto linear = railway::TrackData::Load("linear_track_fixture.txt")->Sample(11);
        Check(linear.displacement[0] == 2 && linear.slope[3] == 1, "Two-point/offset-distance interpolation failed");
        std::cout << "PASS: " << rows << " source nodes, four-channel interpolation/derivatives, delays, lead-in and invalid inputs.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}

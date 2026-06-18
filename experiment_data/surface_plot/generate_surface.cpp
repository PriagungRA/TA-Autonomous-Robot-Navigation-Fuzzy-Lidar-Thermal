// generate_surface.cpp
#include "fuzzy_controller.h"
#include <fstream>
#include <iomanip>

int main() {
    std::ofstream out("surface_data.csv");
    out << "d_obs,trav_eff,speed_scale,rep_gain\n";

    const int N = 41;  // resolusi grid 41x41
    for (int i = 0; i < N; ++i) {
        double d = 2.0 * i / (N - 1);          // d_obs in [0, 2]
        for (int j = 0; j < N; ++j) {
            double t = 1.0 * j / (N - 1);      // trav_eff in [0, 1]
            double speed_scale, rep_gain;
            fuzzy::fuzzyCompute(d, t, speed_scale, rep_gain);
            out << std::fixed << std::setprecision(6)
                << d << "," << t << "," << speed_scale << "," << rep_gain << "\n";
        }
    }
    return 0;
}

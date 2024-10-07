#ifndef FORCE_H
#define FORCE_H

#include <vector>
#include "Agent.h"
#include "Environment.h"

class Force {
public:
    Force(std::vector<Agent>& agents, Environment& environment, double time_constant = 0.5);

    void strat_1(const std::array<double, 2>& point1 = {23.6, 2}, const std::array<double, 2>& point2 = {27, 2});
    void strat_2(const std::array<double, 2>& point1 = {23.6, 2});
    void point_direction_method(const std::array<std::array<double, 2>, 2>& line_points = {{30, 0}, {30, 20}});
    void calculate_repulsive_forces(bool ellipse = true, double eta = 0.2);
    void calculate_wall_force(bool ellipse = true, double eta_alt = 5);

private:
    std::vector<Agent>& agents;
    Environment& environment;
    double time_constant;
    // Other member variables and methods
};

#endif // FORCE_H

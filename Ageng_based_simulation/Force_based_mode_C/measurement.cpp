#include <iostream>
#include <vector>
#include <cmath>
#include "Agent.h"  // Include your Agent class header

// Function to find the average velocity of the test agent
double find_average_velocity(const Agent& test_agent) {
    double t_in = test_agent.memory.at("t_in");
    double t_out = test_agent.memory.at("t_out");
    double x_in = test_agent.memory.at("x_in");
    double x_out = test_agent.memory.at("x_out");

    if (t_out - t_in > 0) {
        double v = std::sqrt(std::pow(x_out - x_in, 2)) / ((t_out - t_in) * 0.01);
        return v;
    }
    return 0.0; // Handle division by zero
}

// Function to find instantaneous density
double find_inst_time_density(const Agent& test_agent, double x_min, double x_max, const std::vector<Agent>& all_agents) {
    double a_tot = 0;
    double l = x_max - x_min;
    
    for (const auto& agent : all_agents) {
        a_tot += 2 * agent.a; // Assuming 'a' represents the radius of the agent
    }
    double rho = a_tot / l;
    return rho;
}

// Function to find averaged density
double find_averaged_density(const std::vector<double>& ins_density_list, const Agent& test_agent, double dt) {
    double t_in = test_agent.memory.at("t_in");
    double t_out = test_agent.memory.at("t_out");
    
    double sum = 0.0;
    for (const auto& density : ins_density_list) {
        sum += density * dt;
    }
    
    if (t_out - t_in > 0) {
        return sum / ((t_out - t_in) * 0.01);
    }
    return 0.0; // Handle division by zero
}

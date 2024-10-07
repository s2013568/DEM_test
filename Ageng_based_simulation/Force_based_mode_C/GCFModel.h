#ifndef GCFMODEL_H
#define GCFMODEL_H

#include <vector>
#include "Agent.h"
#include "Environment.h"
#include "Force.h"

class GCFModel {
public:
    GCFModel(Environment& environment, std::vector<Agent>& agents, 
              double time_constant = 0.5, bool ellipse = true, 
              double eta = 1.0, double x_min = 0, double x_max = 26);

    void calculate_forces();
    void update(double dt);
    void animate(int steps, double dt = 0.1, int interval = 100, const std::string& output_filename = "crowd_simulation.gif", bool show_forces = false);
    void run_simulation(int steps, double dt = 0.1, int log_interval = 10, bool verbose = true);

private:
    Environment& environment;
    std::vector<Agent>& agents;
    Force force;
    double time_constant;
    bool ellipse;
    double eta;
    int current_step;
    double x_min;
    double x_max;
    int agent_count_passed;
    std::vector<int> recorded_step_of_passing;
};

#endif // GCFMODEL_H

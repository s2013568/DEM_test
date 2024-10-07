#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <numeric>  // For std::accumulate
#include "Agent.h"
#include "Environment.h"
#include "Force.h"
#include "GCFModel.h"

// Function to calculate flow rate
double calculate_flow_rate(int step_of_passing) {
    return (60 - 1) / static_cast<double>(step_of_passing);
}

int main() {
    // Define range for bottleneck_height
    std::vector<double> bottleneck_heights;
    for (double h = 0.7; h < 2.6; h += 0.2) {
        bottleneck_heights.push_back(h);
    }

    // Initialize lists to store results
    std::vector<double> flow_rates = {0.009570154095701541};

    // Run simulation for each bottleneck height
    for (const auto& bottleneck_height : bottleneck_heights) {
        // Define environment
        Environment env(30, 4, 2.8, bottleneck_height, {10, 0}, false);

        // Initialize agents
        std::vector<Agent> agents;
        int num_agents = 100;  // Set the number of agents dynamically based on your needs

        // Define available space for agents
        double start_x = 19.1;  // Right boundary
        double end_x = 0.5;     // Left boundary
        double start_y = 0.5;   // Bottom boundary
        double end_y = 3.5;     // Top boundary
        double available_width = start_x - end_x;
        double available_height = end_y - start_y;

        // Dynamically calculate the number of rows and columns
        double aspect_ratio = available_width / available_height;
        int num_columns = static_cast<int>(std::sqrt(num_agents * aspect_ratio));  // Columns based on aspect ratio
        int num_rows = num_agents / num_columns;

        // Calculate spacing dynamically based on available space and number of agents
        double spacing_x = available_width / num_columns;
        double spacing_y = available_height / num_rows;

        // Generate grid of agent positions
        std::vector<double> x_positions(num_columns);
        std::vector<double> y_positions(num_rows);
        for (int i = 0; i < num_columns; ++i) {
            x_positions[i] = start_x - i * spacing_x;
        }
        for (int i = 0; i < num_rows; ++i) {
            y_positions[i] = start_y + i * spacing_y;
        }

        // Place agents uniformly in both x and y directions
        for (const auto& y : y_positions) {
            for (const auto& x : x_positions) {
                if (agents.size() >= num_agents) {
                    break;
                }
                agents.emplace_back(std::array<double, 2>{x, y}, std::array<double, 2>{0.0, 0.0}, 1.0, 0.18, 0.1, 0.53, 1.25, 1.34, false);
            }
            if (agents.size() >= num_agents) {
                break;
            }
        }

        // Create GCFModel instance
        GCFModel gcf_model(env, agents, 0.5, true, 0.2);
        gcf_model.run_simulation(20000, 0.01, 100, true);

        // Calculate flow rate and record it
        if (!gcf_model.recorded_step_of_passing.empty()) {
            int step_of_passing = gcf_model.recorded_step_of_passing[0];
            double flow_rate = calculate_flow_rate(step_of_passing);
            flow_rates.push_back(flow_rate);
        } else {
            flow_rates.push_back(0);
        }
    }

    // Output flow rates
    for (size_t i = 0; i < flow_rates.size(); ++i) {
        std::cout << "Bottleneck Height: " << (0.7 + i * 0.2) << ", Flow Rate: " << flow_rates[i] << std::endl;
    }

    return 0;
}

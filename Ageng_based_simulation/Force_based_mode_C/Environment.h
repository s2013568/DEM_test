#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>

class Environment {
public:
    Environment(double width, double height, double bottleneck_width, 
                double bottleneck_height, const std::array<double, 2>& location = {0, 0}, 
                bool periodic = false);

    bool is_in_bottleneck(const Agent& agent) const;
    bool is_within_walls(const Agent& agent) const;
    bool apply_periodic_boundary(Agent& agent);
    void write_memory(Agent& agent, double x_min, double x_max, int current_time);

    double width;
    double height;
    double bottleneck_width;
    double bottleneck_height;
    std::array<double, 2> location;  // Location offset for the bottleneck's center
    bool periodic;  // Periodic boundary condition flag

    struct Region {
        double x_min, x_max, y_min, y_max;
    };

    Region left_region;
    Region right_region;
    Region bottleneck;
    std::vector<Region> walls; // Store wall definitions
    std::vector<double> ins_density; // Replace with appropriate type for density
};

#endif // ENVIRONMENT_H

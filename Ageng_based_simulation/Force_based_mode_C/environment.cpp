#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <iomanip>

class Agent; // Forward declaration of Agent class

class Environment {
public:
    Environment(double width, double height, double bottleneck_width, 
                double bottleneck_height, std::array<double, 2> location = {0, 0}, 
                bool periodic = false) 
        : width(width), height(height), bottleneck_width(bottleneck_width),
          bottleneck_height(bottleneck_height), location(location), periodic(periodic) {
        
        // Calculate bottleneck's center position based on the location offset
        double center_x = width / 2 + location[0];
        double center_y = height / 2 + location[1];

        // Define the left and right regions and bottleneck
        left_region = {0, center_x - bottleneck_width / 2, 0, height};
        right_region = {center_x + bottleneck_width / 2, width, 0, height};
        bottleneck = {center_x - bottleneck_width / 2, center_x + bottleneck_width / 2,
                      center_y - bottleneck_height / 2, center_y + bottleneck_height / 2};

        ins_density.clear();

        // Initialize walls only if periodic is false
        if (!periodic) {
            walls.push_back({0, 0, 0, height}); // Left wall
            walls.push_back({width, width, 0, height}); // Right wall
            walls.push_back({0, width, 0, 0}); // Bottom wall
            walls.push_back({0, width, height, height}); // Top wall

            if (bottleneck_height != 0 && bottleneck_width != 0) {
                // Left region walls (vertical)
                walls.push_back({left_region.x_max, left_region.x_max, 0, bottleneck.y_min}); // Lower vertical wall
                walls.push_back({left_region.x_max, left_region.x_max, bottleneck.y_max, height}); // Upper vertical wall

                // Right region walls (vertical)
                walls.push_back({right_region.x_min, right_region.x_min, 0, bottleneck.y_min}); // Lower vertical wall
                walls.push_back({right_region.x_min, right_region.x_min, bottleneck.y_max, height}); // Upper vertical wall

                // Bottleneck walls (horizontal)
                walls.push_back({bottleneck.x_min, bottleneck.x_max, bottleneck.y_max, bottleneck.y_max}); // Top bottleneck wall
                walls.push_back({bottleneck.x_min, bottleneck.x_max, bottleneck.y_min, bottleneck.y_min}); // Bottom bottleneck wall
            }
        }
    }

    bool is_in_bottleneck(const Agent& agent) const {
        const auto& pos = agent.position;
        return (bottleneck.x_min <= pos[0] && pos[0] <= bottleneck.x_max &&
                bottleneck.y_min <= pos[1] && pos[1] <= bottleneck.y_max);
    }

    bool is_within_walls(const Agent& agent) const {
        if (periodic) {
            return true; // If periodic, there's no concept of walls
        }

        const auto& pos = agent.position;
        // Check left region
        if (pos[0] <= left_region.x_max && pos[1] <= left_region.y_max) {
            return true;
        }
        // Check right region
        if (pos[0] >= right_region.x_min && pos[1] <= right_region.y_max) {
            return true;
        }
        // Check if in bottleneck
        return is_in_bottleneck(agent);
    }

    bool apply_periodic_boundary(Agent& agent) {
        if (periodic) {
            auto& pos = agent.position;
            if (pos[0] > width || pos[1] > height) {
                pos[0] = fmod(pos[0], width);
                pos[1] = fmod(pos[1], height);
                return true;
            }
            return false;
        }
        return false;
    }

    void write_memory(Agent& agent, double x_min, double x_max, int current_time) {
        if (agent.testing && !agent.tested && agent.memory["t_in"] == -1) {
            agent.memory["t_in"] = current_time;
            agent.memory["x_min"] = x_min;
        } else if (!agent.testing && agent.tested && agent.memory["t_out"] == -1) {
            agent.memory["t_out"] = current_time;
            agent.memory["x_max"] = x_max;
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const Environment& env) {
        os << "Environment(width=" << env.width << ", height=" << env.height
           << ", bottleneck_width=" << env.bottleneck_width
           << ", bottleneck_height=" << env.bottleneck_height
           << ", periodic=" << std::boolalpha << env.periodic << ")";
        return os;
    }

private:
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

// Define Agent class as per your requirements
class Agent {
public:
    std::array<double, 2> position; // Example structure; define as needed
    bool testing = false;
    bool tested = false;
    std::unordered_map<std::string, int> memory = {{"t_in", -1}, {"t_out", -1}, {"x_min", 0}, {"x_max", 0}};
};

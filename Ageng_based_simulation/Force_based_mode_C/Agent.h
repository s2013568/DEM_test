#ifndef AGENT_H
#define AGENT_H

#include <array>
#include <unordered_map>

class Agent {
public:
    Agent(const std::array<double, 2>& position, const std::array<double, 2>& velocity,
          double mass = 1.0, double a_min = 0.18, double b_min = 0.5,
          double desired_walking_speed = 1.3, double tau = 0.53, double f = 3,
          bool ellipse = true, bool test = false);
    
    void move(double dt);
    void reset();
    void reset_wall();
    void add_force();
    
    friend std::ostream& operator<<(std::ostream& os, const Agent& agent);

    std::array<double, 2> position;  // 2D position
    std::array<double, 2> velocity;  // 2D velocity
    double mass;                      // Mass of the agent
    double radius;                    // Radius, representing agent size
    double a_min;                    // Minimum value for a
    double b_min;                    // Minimum value for b
    bool ellipse;                    // Flag for ellipse representation
    double desired_walking_speed;    // Desired walking speed
    double tau;                      // Parameter tau
    double f;                        // Parameter f
    std::array<double, 2> driving_force;   // Driving force
    std::array<double, 2> repulsion_force; // Repulsion force
    std::array<double, 2> wall_force;      // Wall force
    std::array<double, 2> total_force;      // Total force acting on the agent
    double a;                         // Semi-major axis
    double b;                         // Semi-minor axis

    bool stopped;                    // Stopped state
    std::unordered_map<std::string, int> memory;  // Memory of the agent
    bool memory_lock;                // Lock for memory access
    bool tested;                     // Testing state
    bool testing;                    // Testing flag
};

#endif // AGENT_H

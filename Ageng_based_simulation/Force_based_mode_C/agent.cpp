#include <iostream>
#include <array>
#include <cmath>
#include <unordered_map>
#include "Force.h"  // Include your Force class header

class Agent {
public:
    Agent(const std::array<double, 2>& position, const std::array<double, 2>& velocity, 
          double mass = 1.0, double a_min = 0.18, double b_min = 0.5, 
          double desired_walking_speed = 1.3, double tau = 0.53, double f = 3, 
          bool ellipse = true, bool test = false)
        : position(position), velocity(velocity), mass(mass), a_min(a_min), b_min(b_min),
          ellipse(ellipse), desired_walking_speed(desired_walking_speed), tau(tau), f(f), test(test) {

        if (b_min == 0.0) {
            radius = a_min;  // Radius, representing agent size
        }

        // Internal variables initialization
        driving_force = {0.0, 0.0};
        repulsion_force = {0.0, 0.0};
        wall_force = {0.0, 0.0};

        a = a_min + tau * std::hypot(velocity[0], velocity[1]);
        b = f * b_min - ((f - 1) * b_min * std::hypot(velocity[0], velocity[1]) / desired_walking_speed);

        stopped = false;
        
        // Memory initialization
        memory = {
            {"t_in", -1},
            {"t_out", -1},
            {"x_in", 0},
            {"x_out", 26}
        };

        memory_lock = false;
        tested = false;
        testing = false;
    }

    void move(double dt) {
        total_force = driving_force + repulsion_force + wall_force;
        std::array<double, 2> acceleration = {total_force[0] / mass, total_force[1] / mass};
        velocity[0] += acceleration[0] * dt;
        velocity[1] += acceleration[1] * dt;

        // Update the agent's position based on its velocity
        position[0] += velocity[0] * dt;
        position[1] += velocity[1] * dt;

        a = a_min + tau * std::hypot(velocity[0], velocity[1]);
        b = f * b_min - ((f - 1) * b_min * std::hypot(velocity[0], velocity[1]) / desired_walking_speed);
    }

    void reset() {
        repulsion_force = {0.0, 0.0};
    }

    void reset_wall() {
        wall_force = {0.0, 0.0};
    }

    void add_force() {
        total_force = repulsion_force + driving_force;
    }

    friend std::ostream& operator<<(std::ostream& os, const Agent& agent) {
        os << "Agent(pos=[" << agent.position[0] << ", " << agent.position[1] << "], "
           << "vel=[" << agent.velocity[0] << ", " << agent.velocity[1] << "])";
        return os;
    }

private:
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

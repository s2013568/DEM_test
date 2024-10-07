#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include "Agent.h"  // Include your Agent class header
#include "Misc.h"   // Include your Misc class header

class Force {
public:
    Force(std::vector<Agent>& agents, Environment& environment, double time_constant = 0.5)
        : agents(agents), environment(environment), time_constant(time_constant) {}

    // Driving force calculation methods
    void strat_1(const std::array<double, 2>& point1 = {23.6, 2}, const std::array<double, 2>& point2 = {27, 2}) {
        for (auto& agent : agents) {
            const auto& agent_pos = agent.position;
            std::array<double, 2> pointing_vector;

            if (agent_pos[0] < 23.6) {
                pointing_vector = normalize(point1 - agent_pos) * agent.desired_walking_speed;
            } else {
                pointing_vector = normalize(point2 - agent_pos) * agent.desired_walking_speed;
            }
            agent.driving_force = agent.mass * (1 / time_constant) * (pointing_vector - agent.velocity);
        }
    }

    void strat_2(const std::array<double, 2>& point1 = {23.6, 2}) {
        for (auto& agent : agents) {
            const auto& agent_pos = agent.position;
            std::array<double, 2> point2 = {agent_pos[0] + 10.0, agent_pos[1]};
            std::array<double, 2> pointing_vector;

            if (agent_pos[1] >= environment.height / 2) {
                int coefficient = point_position_relative_to_line(agent_pos[0], agent_pos[1], point2[0], point2[1],
                                                                  environment.bottleneck.x_min, environment.bottleneck.y_max);
                if (coefficient >= 0) {
                    pointing_vector = normalize(point2 - agent_pos) * agent.desired_walking_speed;
                } else {
                    pointing_vector = normalize(point1 - agent_pos) * agent.desired_walking_speed;
                }
            } else {
                int coefficient = point_position_relative_to_line(agent_pos[0], agent_pos[1], point2[0], point2[1],
                                                                  environment.bottleneck.x_min, environment.bottleneck.y_min);
                if (coefficient <= 0) {
                    pointing_vector = normalize(point2 - agent_pos) * agent.desired_walking_speed;
                } else {
                    pointing_vector = normalize(point1 - agent_pos) * agent.desired_walking_speed;
                }
            }
            agent.driving_force = agent.mass * (1 / time_constant) * (pointing_vector - agent.velocity);
        }
    }

    void point_direction_method(const std::array<std::array<double, 2>, 2>& line_points = {{30, 0}, {30, 20}}) {
        std::array<double, 2> point1 = line_points[0];
        std::array<double, 2> point2 = line_points[1];

        for (auto& agent : agents) {
            const auto& agent_pos = agent.position;
            std::array<double, 2> line_vector = point2 - point1;
            std::array<double, 2> point_to_agent_vector = agent_pos - point1;

            double line_length_squared = dot(line_vector, line_vector);
            std::array<double, 2> pointing_vector;

            if (line_length_squared == 0) {
                pointing_vector = normalize(point1 - agent_pos);
            } else {
                double t = dot(point_to_agent_vector, line_vector) / line_length_squared;
                t = std::max(0.0, std::min(1.0, t));
                std::array<double, 2> nearest_point_on_line = point1 + t * line_vector;
                pointing_vector = normalize(nearest_point_on_line - agent_pos) * agent.desired_walking_speed;
            }

            agent.driving_force = agent.mass * (1 / time_constant) * (pointing_vector - agent.velocity);
        }
    }

    // Repulsive force calculation
    void calculate_repulsive_forces(bool ellipse = true, double eta = 0.2) {
        double tau = 1.0;
        double reps = 0.1;
        double rc = 2;

        auto apply_periodic_distance = [&](const std::array<double, 2>& agent_i_pos, const std::array<double, 2>& agent_j_pos) {
            std::array<double, 2> dx = {agent_j_pos[0] - agent_i_pos[0], agent_j_pos[1] - agent_i_pos[1]};

            if (std::abs(dx[0]) > environment.width / 2) {
                dx[0] -= std::copysign(environment.width, dx[0]);
            }
            if (std::abs(dx[1]) > environment.height / 2) {
                dx[1] -= std::copysign(environment.height, dx[1]);
            }

            return dx;
        };

        if (!ellipse) {
            for (size_t i = 0; i < agents.size(); ++i) {
                agents[i].reset();
                for (size_t j = 0; j < agents.size(); ++j) {
                    if (i != j) {
                        double separation = std::hypot(agents[j].position[0] - agents[i].position[0], 
                                                       agents[j].position[1] - agents[i].position[1]);
                        std::array<double, 2> separation_unit_vector = {(agents[j].position[0] - agents[i].position[0]) / separation,
                                                                        (agents[j].position[1] - agents[i].position[1]) / separation};

                        double relative_velocity = 0.5 * (dot(agents[i].velocity - agents[j].velocity, separation_unit_vector) + 
                                                           std::abs(dot(agents[i].velocity - agents[j].velocity, separation_unit_vector)));

                        double reduced_vision_factor = (std::hypot(agents[i].velocity[0], agents[i].velocity[1]) != 0) 
                            ? (dot(agents[i].velocity, separation_unit_vector) + 
                               std::abs(dot(agents[i].velocity, separation_unit_vector))) / 
                               (2 * std::hypot(agents[i].velocity[0], agents[i].velocity[1])) 
                            : 0;

                        agents[i].repulsion_force += - (agents[i].mass * reduced_vision_factor * 
                            std::pow((eta * agents[i].desired_walking_speed + relative_velocity), 2) / 
                            (separation - (agents[i].a + tau * agents[i].velocity[0]) - 
                            (agents[j].a + tau * agents[i].velocity[0]))) * separation_unit_vector;
                    }
                }
            }
        } else {
            for (size_t i = 0; i < agents.size(); ++i) {
                agents[i].reset();
                for (size_t j = 0; j < agents.size(); ++j) {
                    if (i != j) {
                        std::array<double, 2> separation_vector = apply_periodic_distance(agents[i].position, agents[j].position);
                        double separation = std::hypot(separation_vector[0], separation_vector[1]);
                        std::array<double, 2> separation_unit_vector = {separation_vector[0] / separation, separation_vector[1] / separation};
                        double relative_velocity = 0.5 * (dot(agents[i].velocity - agents[j].velocity, separation_unit_vector) + 
                                                           std::abs(dot(agents[i].velocity - agents[j].velocity, separation_unit_vector)));

                        double reduced_vision_factor = (std::hypot(agents[i].velocity[0], agents[i].velocity[1]) != 0) 
                            ? (dot(agents[i].velocity, separation_unit_vector) + 
                               std::abs(dot(agents[i].velocity, separation_unit_vector))) / 
                               (2 * std::hypot(agents[i].velocity[0], agents[i].velocity[1])) 
                            : 0;

                        double d = closest_distance_between_ellipses(agents[i], agents[j], environment);
                        if (d < reps) {
                            double l_hat = calculate_closest_distance(agents[i], agents[j]);
                            double Frep = (agents[i].mass * std::pow((eta * agents[i].desired_walking_speed + relative_velocity), 2) / reps);
                            agents[i].repulsion_force -= (((-2 * Frep) / (reps - l_hat)) * d + 3 * Frep) * separation_unit_vector;
                        } else if (d > rc) {
                            agents[i].repulsion_force += 0;
                        } else if (d > rc - reps && d < rc) {
                            double Fc = (agents[i].mass * reduced_vision_factor * 
                                          std::pow((eta * agents[i].desired_walking_speed + relative_velocity), 2) / (rc - reps));
                            agents[i].repulsion_force -= ((Fc) / (rc - reps)) * (d - rc + reps) * separation_unit_vector;
                        } else {
                            agents[i].repulsion_force += - (agents[i].mass * reduced_vision_factor * 
                                                            std::pow((eta * agents[i].desired_walking_speed + relative_velocity), 2) / d) * separation_unit_vector;
                        }
                    }
                }
            }
        }
    }

    void calculate_wall_force(bool ellipse = true, double eta_alt = 5) {
        for (auto& agent : agents) {
            agent.reset_wall();
            double D_mag = std::numeric_limits<double>::max();
            std::array<double, 2> D = {0, 0};
            std::array<double, 2> P = {0, 0};
            std::array<double, 2> A = {0, 0};
            std::array<double, 2> B = {0, 0};
            std::pair<double, double> wallxy = {0, 0};

            for (size_t i = 0; i < environment.walls.size(); ++i) {
                const auto& wall = environment.walls[i];
                auto x_range = wall.x_range;
                auto y_range = wall.y_range;
                std::array<std::pair<double, double>, 2> pos = {{x_range[0], y_range[0]}, {x_range[1], y_range[1]}};
                double d_mag, d_unit, p, a, b;

                std::tie(d_mag, d_unit, p, a, b) = calculate_shortest_distance_between_point_and_line(agent, pos);

                if (d_mag < D_mag) {
                    D_mag = d_mag;
                    D = d_unit;
                    P = p;
                    A = a;
                    B = b;
                    wallxy = {x_range[0], x_range[1]};
                    if (det(A, B) < 0) {
                        A = b;
                        B = a;
                    }
                }
            }

            // Transform the line vector into the frame of the ellipse
            std::array<double, 2> N = {-D[1], D[0]};
            std::array<double, 2> P_dash = {-agent.a * N[0], -agent.b * N[1]};
            double b_value = dot(N, P_dash) - det(A, B);

            double Theta_R = (D[1] == 0) ? (D[0] > 0 ? -M_PI / 2 : M_PI / 2) : std::atan(-((agent.b * D[0]) / (agent.a * D[1])));
            std::array<double, 2> R = {agent.a * std::cos(Theta_R), agent.b * std::sin(Theta_R)};
            double d_value = dot(N, R) - det(A, B);
            double l = b_value - d_value;

            double reduced_vision_factor = (std::hypot(agent.velocity[0], agent.velocity[1]) != 0) 
                ? (dot(agent.velocity, D) + std::abs(dot(agent.velocity, N))) / (2 * std::hypot(agent.velocity[0], agent.velocity[1])) 
                : 0;

            double alpha = std::acos(dot(normalize(P - agent.position), normalize(agent.velocity)));
            double q = (std::cos(alpha) * std::cos(alpha) / (agent.a * agent.a)) + (std::sin(alpha) * std::sin(alpha) / (agent.b * agent.b));
            double r = std::sqrt(1 / q);
            double biw = (1 - D_mag / (l + r)) * std::heaviside(1 - D_mag / (l + r), 1);

            std::array<double, 2> f = {eta_alt * agent.desired_walking_speed * reduced_vision_factor * biw * normalize(P - agent.position)[0],
                                        eta_alt * agent.desired_walking_speed * reduced_vision_factor * biw * normalize(P - agent.position)[1]};

            agent.wall_force -= f;
        }
    }

private:
    std::vector<Agent>& agents;
    Environment& environment;
    double time_constant;

    // Helper functions
    std::array<double, 2> normalize(const std::array<double, 2>& vector) {
        double norm = std::hypot(vector[0], vector[1]);
        return (norm == 0) ? std::array<double, 2>{0, 0} : std::array<double, 2>{vector[0] / norm, vector[1] / norm};
    }

    double dot(const std::array<double, 2>& a, const std::array<double, 2>& b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    // Implement other helper functions as needed...
};


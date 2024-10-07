#ifndef MISC_H
#define MISC_H

#include <array>
#include "Agent.h"
#include "Environment.h"

// Function declarations
std::array<double, 2> normalize(const std::array<double, 2>& v);
std::array<double, 2> apply_periodic_distance(const std::array<double, 2>& agent_i_pos, 
                                               const std::array<double, 2>& agent_j_pos, 
                                               const Environment& env);
double closest_distance_between_ellipses(const Agent& agent1, const Agent& agent2, const Environment& env);
double calculate_angle(const std::array<double, 2>& velocity);
std::tuple<double, std::array<double, 2>, std::array<double, 2>, std::array<double, 2>, std::array<double, 2>>
calculate_shortest_distance_between_point_and_line(const Agent& agent, 
                                                    const std::array<std::array<double, 2>, 2>& line_points);
double det_numpy(const std::array<double, 2>& A, const std::array<double, 2>& B);
double contact_distance_between_two_ellipses(const Agent& agent1, const Agent& agent2);
double calculate_closest_distance(const Agent& agent1, const Agent& agent2);

#endif // MISC_H

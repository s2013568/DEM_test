#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include "Agent.h"  // Include your Agent class header
#include "Environment.h" // Include your Environment class header

// Function to normalize a vector
std::array<double, 2> normalize(const std::array<double, 2>& v) {
    double norm = std::hypot(v[0], v[1]);
    if (norm == 0) {
        return v;  // Return the original vector if the norm is 0
    }
    return {v[0] / norm, v[1] / norm};
}

// Function to apply periodic distance between two agent positions
std::array<double, 2> apply_periodic_distance(const std::array<double, 2>& agent_i_pos, const std::array<double, 2>& agent_j_pos, const Environment& env) {
    std::array<double, 2> dx = {agent_j_pos[0] - agent_i_pos[0], agent_j_pos[1] - agent_i_pos[1]};

    // Apply periodic boundaries along the x-axis
    if (std::abs(dx[0]) > env.width / 2) {
        dx[0] -= std::copysign(env.width, dx[0]);
    }

    // Apply periodic boundaries along the y-axis
    if (std::abs(dx[1]) > env.height / 2) {
        dx[1] -= std::copysign(env.height, dx[1]);
    }

    return dx;
}

// Function to calculate the closest distance between two ellipses
double closest_distance_between_ellipses(const Agent& agent1, const Agent& agent2, const Environment& env) {
    auto separation_unit_vector = normalize(apply_periodic_distance(agent1.position, agent2.position, env));
    auto velocity_unit_vector1 = normalize(agent1.velocity);
    auto velocity_unit_vector2 = normalize(agent2.velocity);
    double angle1 = std::acos(dot(velocity_unit_vector1, separation_unit_vector));
    double angle2 = std::acos(dot(velocity_unit_vector2, separation_unit_vector));

    double q1 = (std::cos(angle1) * std::cos(angle1) / (agent1.a * agent1.a)) + (std::sin(angle1) * std::sin(angle1) / (agent1.b * agent1.b));
    double q2 = (std::cos(angle2) * std::cos(angle2) / (agent2.a * agent2.a)) + (std::sin(angle2) * std::sin(angle2) / (agent2.b * agent2.b));

    double r1 = std::sqrt(1 / q1);
    double r2 = std::sqrt(1 / q2);
    
    double h = std::hypot(agent1.position[0] - agent2.position[0], agent1.position[1] - agent2.position[1]) - r1 - r2;
    
    return h;
}

// Function to calculate the angle from a velocity vector
double calculate_angle(const std::array<double, 2>& velocity) {
    return std::degrees(std::atan2(velocity[1], velocity[0]));
}

// Function to calculate the shortest distance between a point and a line
std::tuple<double, std::array<double, 2>, std::array<double, 2>, std::array<double, 2>, std::array<double, 2>>
calculate_shortest_distance_between_point_and_line(const Agent& agent, const std::array<std::array<double, 2>, 2>& line_points) {
    std::array<double, 2> point1 = line_points[0];
    std::array<double, 2> point2 = line_points[1];
    std::array<double, 2> agent_pos = agent.position;
    std::array<double, 2> line_vector = {point2[0] - point1[0], point2[1] - point1[1]};
    double line_length_squared = dot(line_vector, line_vector);
    std::array<double, 2> point_to_agent_vector = {agent_pos[0] - point1[0], agent_pos[1] - point1[1]};
    double t = dot(point_to_agent_vector, line_vector) / line_length_squared;
    t = std::clamp(t, 0.0, 1.0);  // Clamp t between 0 and 1

    std::array<double, 2> nearest_point_on_line = {point1[0] + t * line_vector[0], point1[1] + t * line_vector[1]};
    std::array<double, 2> d = {nearest_point_on_line[0] - agent_pos[0], nearest_point_on_line[1] - agent_pos[1]};
    double d_mag = std::hypot(d[0], d[1]);
    std::array<double, 2> d_unit = normalize(d);

    return {d_mag, d_unit, nearest_point_on_line, point1, point2};
}

// Function to calculate the determinant of a 2x2 matrix formed by two vectors A and B
double det_numpy(const std::array<double, 2>& A, const std::array<double, 2>& B) {
    return A[0] * B[1] - A[1] * B[0];
}

// Function to calculate the contact distance between two ellipses (to be implemented)
double contact_distance_between_two_ellipses(const Agent& agent1, const Agent& agent2) {
    // Placeholder function; actual implementation needed
    std::array<double, 2> k1 = normalize(agent1.velocity);
    double b1 = agent1.b;
    double a1 = agent1.a;
    double eta = (a1 / b1) - 1;

    // Implement the necessary transformation and distance calculations
    // ...
}

// Function to calculate the closest distance between two agents
double calculate_closest_distance(const Agent& agent1, const Agent& agent2) {
    double a1 = agent1.a;
    double b1 = agent1.b;
    double a2 = agent2.a;
    double b2 = agent2.b;

    std::array<double, 2> k1 = normalize(agent1.velocity);
    std::array<double, 2> k2 = normalize(agent2.velocity);
    
    double e1 = std::sqrt(1 - (b1 * b1 / (a1 * a1)));
    double e2 = std::sqrt(1 - (b2 * b2 / (a2 * a2)));

    auto transformation_matrix = [](double a, double b, const std::array<double, 2>& k) {
        std::array<std::array<double, 2>, 2> T;
        double k_k = k[0] * k[0] + k[1] * k[1];
        T[0][0] = (1.0 / b) * (1.0 + (b / a - 1) * k_k);
        T[0][1] = 0; // Modify as needed
        T[1][0] = 0; // Modify as needed
        T[1][1] = (1.0 / b) * (1.0 + (b / a - 1) * k_k);
        return T;
    };

    std::array<std::array<double, 2>, 2> T1 = transformation_matrix(a1, b1, k1);
    
    // Step 2: Transform the second ellipse
    std::array<double, 2> d_hat = normalize({T1[0][0] * k2[0], T1[1][0] * k2[1]});
    
    // Step 3: Apply transformed distances
    auto distance_transformed = [](double a2, double b2, double e1, const std::array<double, 2>& d_hat) {
        return (1.0 / b1) * (1.0 / std::sqrt(1 - e1 * e1 * dot(d_hat, k1) * dot(d_hat, k1)));
    };

    double d_transformed = distance_transformed(a2, b2, e1, d_hat);
    
    // Inverse transformation to get the actual distance
    double distance = d_transformed / std::sqrt(dot(T1[0], T1[0]) + dot(T1[1], T1[1]));
    
    return distance;
}

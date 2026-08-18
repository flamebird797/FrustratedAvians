#include "proportional_nav.hpp"
double closing_velocity(const Vector& bird_pos, const Vector& bird_vel, const Vector& target_pos, const Vector& target_vel) {
    Vector relative_position = target_pos - bird_pos;
    Vector relative_velocity = target_vel - bird_vel;
    
    Vector unit_relative_position = relative_position.normalized();
    
   
    double closing_vel = -relative_velocity.dot(unit_relative_position);
    
    return closing_vel;
}
double line_of_sight_rate(const Vector& bird_pos, const Vector& bird_vel, const Vector& target_pos, const Vector& target_vel) {
    Vector relative_position = target_pos - bird_pos;
    Vector relative_velocity = target_vel - bird_vel;

    double distance_squared = relative_position.dot(relative_position);
    if (distance_squared < 1e-10) return 0.0; // Avoid division by zero

    double los_rate = (relative_position.cross(relative_velocity)) / distance_squared;
    
    return los_rate;
}

double proportional_navigation_acceleration(const Vector& bird_pos, const Vector& bird_vel, const Vector& target_pos, const Vector& target_vel, double navigation_constant) {
    double closing_vel = closing_velocity(bird_pos, bird_vel, target_pos, target_vel);
    double los_rate = line_of_sight_rate(bird_pos, bird_vel, target_pos, target_vel);

    return navigation_constant * closing_vel * los_rate;
}
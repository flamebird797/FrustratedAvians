#include "integrator.hpp"
void euler_integration(double dt, Vector& position, Vector& velocity, const Vector& acceleration) {
    // Update position
    position = position + velocity * dt + 0.5 * acceleration * dt * dt;

    // Update velocity
    velocity = velocity + acceleration * dt;  
}

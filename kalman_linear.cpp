#include "kalman_linear.h"

Kalman_Linear::Kalman_Linear(double dt) : dt(dt)
{}

/// update state based on dynamic equation
std::vector<double> Kalman_Linear::compute_state_predict(const std::vector<double>& x0)
{
    // update position
    this->x0[0] = x0[0] + dt * x0[1] + dt * dt / 2 * x0[2];
    // update velocity
    this->x0[1] = x0[1] + dt * x0[2];
    // update acceleration
    this->x0[2] = x0[2];
    return this->x0;
}
/// update state based on Kalman filter prediction
std::vector<double> Kalman_Linear::compute_state_update(const std::vector<double>&& yt)
{
    // update position
    x1[0] = x0[0] + gain[0] * (yt[0] - x0[0]);
    x1[1] = x0[1] + gain[1] * (yt[0] - x0[0]) / dt;
    x1[2] = x0[2] + gain[2] * (yt[0] - x0[0]) / (dt * dt * 0.5);
    return this->x1;
}
/// update gain
std::vector<double> Kalman_Linear::compute_gain(const std::vector<double>& g)
{
    return this->gain;
}
/// set gain
void Kalman_Linear::set_gain(double alpha, double beta, double gama)
{
    gain[0] = alpha, gain[1] = beta, gain[2] = gama;
}


#ifndef KALMAN_LINEAR_H
#define KALMAN_LINEAR_H

#include <vector>
class Kalman_Linear
{
public:
    explicit Kalman_Linear(double dt = 1.);
    std::vector<double> compute_state_predict(const std::vector<double>& x0);
    std::vector<double> compute_state_update(const std::vector<double>&& yt);
    std::vector<double> compute_gain(const std::vector<double>& g);
    void set_gain(double alpha = 1, double beta = 1, double gama = 1);
private:
    /// 2d state variable x, xdot,
    /// x0 = predicted state based on
    /// state extrapolation equation / dynamical equation
    std::vector<double> x0{0,0,0};
    /// x1 = state update equation based on Kalman filter
    std::vector<double> x1{0,0,0};
    /// gain factors
    std::vector<double> gain{0,0,0};
    /// dt = scaling factor
    double dt{0};
};

#endif // KALMAN_LINEAR_H

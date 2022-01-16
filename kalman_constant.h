#ifndef KALMAN_CONSTANT_H
#define KALMAN_CONSTANT_H


class Kalman_Constant
{
public:
    Kalman_Constant(double sigma = 0, double q = 0)
        : sigma(sigma), q(q)
    {};
    double compute_state_predict(double x0);
    double compute_uncertainty_predict(double p0);
    double compute_gain();
    double compute_state_update(double xt);
    double compute_uncertainty_estimate();
private:
    double x0{0}, p0{0};
    double x1{0}, p1{0};
    double gain{0};
    // standard error
    double sigma{0};
    // process noice variance
    double q{0};
};

#endif // KALMAN_CONSTANT_H

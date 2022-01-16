#ifndef KALMAN_CONSTANT_H
#define KALMAN_CONSTANT_H


class Kalman_Constant
{
public:
    Kalman_Constant(double x0 = 0, double gain = 1.)
        : gain(gain)
    {};
    double compute_state_predict(double x0);
    double compute_state_update(double x0);
    double compute_gain(double n);
private:
    double x0{0};
    double x1{0};
    double gain{0};
};

#endif // KALMAN_CONSTANT_H

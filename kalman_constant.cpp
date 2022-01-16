#include "kalman_constant.h"

double Kalman_Constant::compute_state_predict(double x0)
{
    return this->x0 = x0;
}

double Kalman_Constant::compute_state_update(double yt)
{
    return x1 = x0 + gain * (yt - x0);
}

double Kalman_Constant::compute_gain(double n)
{
    return gain = 1./n;
}

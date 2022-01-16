#include "kalman_constant.h"
#include <math.h>

double Kalman_Constant::compute_state_predict(double x0)
{
    return this->x0 = x0;
}
double Kalman_Constant::compute_uncertainty_predict(double p0)
{
    return this->p0 = p0 + q;
}
double Kalman_Constant::compute_gain()
{
    return gain = p0 / (p0 + sigma * sigma);
}
double Kalman_Constant::compute_state_update(double xt)
{
    return x1 = x0 + gain * (xt - x0);
}
double Kalman_Constant::compute_uncertainty_estimate()
{
    return p1 = (1 - gain) * p0;
}


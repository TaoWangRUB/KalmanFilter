#include "matplotlibcpp.h"
#include <vector>
#include <random>
#include <iostream>

#include "kalman_constant.h"
#include "kalman_linear.h"

const int MAXN = 10;

namespace plt = matplotlibcpp;

void build_kalman_constant(void);
void build_kalman_linear(void);
int main() {
    /// build 1d constant model
    build_kalman_constant();
    /// build 2d linear model
    //build_kalman_linear();
}

void build_kalman_constant()
{
    double x0 = 10., p0 = 100*100.;
    double sigma = 0.1, q = 0.15;
    //std::vector<double> yt{48.54, 47.11, 55.01, 55.15, 49.89, 40.85, 46.72, 50.05, 51.27, 49.95};
    //std::vector<double> y0{49.979, 50.025, 50, 50.003, 49.994, 50.002, 49.999, 50.006, 49.998, 49.991};
    //std::vector<double> yt{49.95, 49.967, 50.1, 50.106, 49.992, 49.819, 49.933, 50.007, 50.023, 49.99};
    std::vector<double> y0{50.479, 51.025, 51.5, 52.003, 52.494, 53.002, 53.499, 54.006, 54.498, 54.991};
    std::vector<double> yt{50.45, 50.967, 51.6, 52.106, 52.492, 52.819, 53.433, 54.007, 54.523, 54.99};
    Kalman_Constant model(sigma, q);
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(1010,100.0);

    std::vector<double> tn, yPred, yEsti, yMeas, gn, pn;
    int t = 0;
    while (t < MAXN) {
        t++;
        auto xt = model.compute_state_predict(x0);
        //auto yt = distribution(generator);
        auto pt = model.compute_uncertainty_predict(p0);
        auto gain = model.compute_gain();
        x0 = model.compute_state_update(yt[t-1]);
        p0 = model.compute_uncertainty_estimate();
        tn.push_back(t);
        yPred.push_back(xt);
        yMeas.push_back(yt[t-1]);
        yEsti.push_back(x0);
        pn.push_back(p0);
        gn.push_back(gain);
    }
    // plot measure / estimate result
    //plt::plot(tn, yPred, {{"marker", "o"}, {"fillstyle", "none"}, {"label", "predict"}});
    plt::plot(tn, yMeas, {{"marker", "^"}, {"fillstyle", "none"}, {"label", "measure"}});
    plt::plot(tn, yEsti, {{"marker", "s"}, {"fillstyle", "none"}, {"label", "estimat"}});
    //plt::axhline(50, {{"c", "red"}, {"ls", "-"}, {"label", "true"}});
    plt::plot(tn, y0, {{"c", "red"}, {"ls", "-"}, {"label", "true"}});
    // Set x-axis limit
    //plt::xlim(0, 1000*1000);
    // Add graph title
    plt::title("Kalman Filter");
    // Enable legend.
    plt::legend();
    //plt::show();
    plt::savefig("./comp_1d.pdf");
    plt::clf();
    // plot gain
    plt::plot(tn, gn, {{"marker", "o"}, {"markerfacecolor", "None"}, {"label", "gain"}});
    plt::title("Kalman Filter gain");
    //plt::show();
    plt::savefig("./gain_1d.pdf");
    plt::clf();
    // plot estimate uncertainty
    plt::plot(tn, pn, {{"marker", "o"}, {"markerfacecolor", "None"}, {"label", "estimate"}});
    plt::axhline(0.01, {{"c", "red"}, {"ls", "-"}, {"label", "estimate"}});
    plt::title("Kalman Filter uncertainty");
    //plt::show();
    plt::savefig("./uncertainty_1d.pdf");
}

void build_kalman_linear()
{
    double dt = 5; // unit s
    double alpha = 0.5, beta = 0.4, gama = 0.1;
    double s0 = 30000., v0 = 50., a0 = 8;
    Kalman_Linear model(dt);
    model.set_gain(alpha, beta, gama);
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0,200.0);
    /// for constant speed input
    //std::vector<double> y{30110, 30265, 30740, 30750, 31135, 31015, 31180, 31610, 31960, 31865};
    std::vector<double> y{30160, 30365, 30890, 31050, 31785, 32215, 33130, 34510, 36010, 37265};
    std::vector<double> tn, ss, sv, sa, sT;
    std::vector<std::vector<double>> sn(3), xdotn(3);
    int t = 0;
    std::vector<double> x0{s0, v0, 0.};
    while (t < MAXN) {
        t++;
        // apply dynamic modle to predict
        auto xn = model.compute_state_predict(x0);

        /// get measurement
        //auto yt = s0 + v0 * t * dt + distribution(generator);
        //auto gain = model.compute_gain(t);
        //x0 = model.compute_state_update({yt, 0});

        // apply Kalman filter to correct
        x0 = model.compute_state_update({y[t-1], 0, 0});
        tn.push_back(t);
        // predict result
        for(int i = 0; i < 3; ++i)
            sn[i].push_back(xn[i]);
        // true and meansured result
        //auto yTrue = s0 + v0 * t * dt;
        auto yTrue = t*dt> 15? s0 + v0 * 15 + v0 * (t * dt - 15) + a0 * pow(t * dt - 15, 2.) / 2.\
                             : s0 + v0 * t * dt;
        sT.push_back(y[t-1]);
        // updated result
        for(int i = 0; i < 3; ++i)
            xdotn[i].push_back(x0[i]);
        ss.push_back(yTrue);
        sv.push_back(t*dt> 15? v0+a0*(t * dt - 15):v0);
        sa.push_back(t*dt> 15? a0:0);
    }
    // plot measure / estimate result
    plt::plot(tn, sT, {{"marker", "o"}, {"fillstyle", "none"}, {"label", "measured"}});
    plt::plot(tn, xdotn[0], {{"marker", "s"}, {"fillstyle", "none"}, {"label", "estimat"}});
    plt::plot(tn, sn[0], {{"marker", "^"}, {"fillstyle", "none"}, {"label", "prediction"}});
    plt::plot(tn, ss, {{"c", "red"}, {"ls", "-."}, {"label", "true"}});
    // Set x-axis limit
    //plt::xlim(0, 1000*1000);
    // Add graph title
    plt::title("Kalman Filter: position");
    // Enable legend.
    plt::legend();
    //plt::show();
    plt::savefig("./x.pdf");
    plt::clf();
    // plot velocity
    plt::plot(tn, sn[1], {{"marker", "o"}, {"fillstyle", "none"}, {"label", "measured"}});
    plt::plot(tn, xdotn[1], {{"marker", "s"}, {"fillstyle", "none"}, {"label", "estimat"}});
    plt::plot(tn, sv, {{"c", "red"}, {"ls", "-."}, {"label", "true"}});
    // Enable legend.
    plt::legend();
    plt::title("Kalman Filter : velocity");
    //plt::show();
    plt::savefig("./xdot.pdf");
    // plot acceleration
    plt::clf();
    plt::plot(tn, sn[2], {{"marker", "o"}, {"fillstyle", "none"}, {"label", "measured"}});
    plt::plot(tn, xdotn[2], {{"marker", "s"}, {"fillstyle", "none"}, {"label", "estimat"}});
    plt::plot(tn, sa, {{"c", "red"}, {"ls", "-."}, {"label", "true"}});
    // Enable legend.
    plt::legend();
    plt::title("Kalman Filter : acceleration");
    //plt::show();
    plt::savefig("./xdotdot.pdf");
}

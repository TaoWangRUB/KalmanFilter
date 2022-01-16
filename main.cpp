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
    //build_kalman_constant();
    /// build 2d linear model
    build_kalman_linear();
}

/*void build_kalman_constant()
{
    double w = 1000.;
    Kalman_Constant model(1.);
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(1010,100.0);

    std::vector<double> tn, xn, yn, gn;
    int t = 0;
    while (t < MAXN) {
        t++;
        auto wt = model.compute_state_predict(w);
        auto yt = distribution(generator);
        auto gain = model.compute_gain(t);
        w = model.compute_state_update(yt);
        tn.push_back(t);
        xn.push_back(w);
        yn.push_back(yt);
        gn.push_back(gain);
    }
    // plot measure / estimate result
    plt::plot(tn, yn, {{"marker", "o"}, {"label", "measured"}});
    plt::plot(tn, xn, {{"label", "estimat"}});
    plt::axhline(1010, {{"c", "red"}, {"ls", "-"}});
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
}
*/
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

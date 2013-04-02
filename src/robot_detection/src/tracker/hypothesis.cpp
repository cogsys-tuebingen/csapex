/// COMPONENT
#include "hypothesis.h"

/// PROJECT
#include <common/global.hpp>
#include <data/angle.h>

/// SYSTEM
#include <Eigen/LU>

Hypothesis* Hypothesis::NULL_HYPOTHESIS(new Hypothesis(Eigen::VectorXd(), Eigen::MatrixXd(), 0, 0));

Eigen::MatrixXd Hypothesis::Q(6,6);
Eigen::MatrixXd Hypothesis::R(3,3);
Eigen::MatrixXd Hypothesis::H(3,6);
Eigen::MatrixXd Hypothesis::I(6,6);

long Hypothesis::next_id = 0;

bool Hypothesis::initialized = initializeConstants();

bool Hypothesis::initializeConstants()
{
    Q.setIdentity();
    Q(0, 0) = std::pow(0.25, 2);
    Q(1, 1) = std::pow(0.25, 2);
    Q(2, 2) = std::pow(0.25, 2);
    Q(3, 3) = std::pow(0.5, 2);
    Q(4, 4) = std::pow(0.5, 2);
    Q(5, 5) = std::pow(0.5, 2);

    R.setZero();
    R(0, 0) = std::pow(1.0, 2);
    R(1, 1) = std::pow(1.0, 2);
    R(2, 2) = std::pow(2.0, 2);

    H.setIdentity();

    I.setIdentity();

    return true;
}

Hypothesis::Hypothesis(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, double decay_time, double decay_per_update)
    : x(x), P(P), decay_max(decay_time), decay_step(decay_per_update), decay(decay_per_update), no_times_updated(0), id(next_id++)
{
    angular_components_.push_back(2);
}

double Hypothesis::probability(const Eigen::Vector3d& pose)
{
    int k = 3;
    Eigen::Vector3d y = pose - x.head<3>();
    Eigen::MatrixXd Ps = P.block(0,0, 3,3);

    double normalizer = std::sqrt(pow(2 * M_PI, k) * Ps.determinant());
    double exponent = -0.5 * y.transpose() * Ps.inverse() * y;

    return std::exp(exponent) / normalizer;
}

bool Hypothesis::predict(double dt)
{
    const Eigen::MatrixXd& F = makeF(dt);

    // x_{k | k-1} = F_k * x_{k-1 | k-1} + B_k * u_k
    //    u_k not known / no control input
    x = F * x;

    // P_{k | k-1} = F_k * P_{k-1 | k-1} * F_k^T + Q_k
    P = F * P * F.transpose() + Q;

    decay -= dt;

    return decay > 0;
}

void Hypothesis::update(const Eigen::Vector3d& measurement)
{
    const Eigen::Vector3d& z = measurement;

    // y_k = z_k - H_k * x_{k | k-1}
    Eigen::Vector3d y = z - H * x;

    // fix rotational components -> interval ]-pi, pi]
    for(std::vector<int>::const_iterator it = angular_components_.begin(); it != angular_components_.end(); ++it) {
        y(*it) = Angle(y(*it)).toRadians();
    }

    // S_k = H_k * P_{k | k-1} * H_k^T + R_k
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // K_k = P_{k | k-1} * H_k^T * S_k^-1
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

//    WARN("P=\n" << P);
//    WARN("P HT=\n" << P * H.transpose());
//    WARN("S1=\n" << S.inverse());
//    WARN("K=\n" << K);

    // x_{k | k} = x_{k | k-1} + K_k * y_k
    x = x + K * y;

    // fix rotational components -> interval ]-pi, pi]
    for(std::vector<int>::const_iterator it = angular_components_.begin(); it != angular_components_.end(); ++it) {
        y(*it) = Angle(y(*it)).toRadians();
    }

    // P_{k | k} = (I - K_k * H_k) * P_{k | k-1}
    P = (I - K * H) * P;

    no_times_updated++;

    decay += decay_step;
    if(decay > decay_max) {
        decay = decay_max;
    }
}

Eigen::MatrixXd Hypothesis::makeF(double dt)
{
    /*
     * 1 0 0 dt 0  0
     * 0 1 0 0  dt 0
     * 0 0 1 0  0  dt
     * 0 0 0 1  0  0
     * 0 0 0 0  1  0
     * 0 0 0 0  0  1
     */
    Eigen::MatrixXd F(6, 6);
    F.setIdentity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    //F(2, 5) = dt;
    return F;
}

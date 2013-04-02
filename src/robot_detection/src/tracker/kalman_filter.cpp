/// HEADER
#include "kalman_filter.h"

/// COMPONENT
#include "hypothesis.h"

KalmanFilter::KalmanFilter()
{
}

Hypothesis* KalmanFilter::getBestHypothesis(const Eigen::Vector3d& pose)
{
    double best_prob = 0.0;
    int best_index = -1;

    double sigma = 2.0;

    for(int i = 0, n = hypotheses_.size(); i < n; ++i) {
        Hypothesis* hyp = hypotheses_[i];
        double distance = hyp->probability(pose);

        if(distance > best_prob) {
            best_prob = distance;
            best_index = i;
        }
    }

    if(best_index == -1) {
        return Hypothesis::NULL_HYPOTHESIS;

    } else {
        // check if the best hypothesis is "near enough"
        Hypothesis* best = hypotheses_[best_index];

        Eigen::Vector2d p = best->getState().head(2);
        Eigen::Vector2d m = pose.head(2);

        double dx = p(0) - m(0);
        double dy = p(1) - m(1);

        double var_x = best->getCovariance()(0,0);
        double var_y = best->getCovariance()(1,1);

        if(dx* dx <= sigma* var_x && dy* dy <= sigma * var_y) {
            // both the x and the y component are within <sigma> standard deviations of the mean => hypothesis is ok
            //   -> dx < sigma * sqrt(var_x) = sigma * std_dev_x
            //   -> dy < sigma * sqrt(var_y) = sigma * std_dev_y
            return best;
        } else {
            return Hypothesis::NULL_HYPOTHESIS;
        }
    }
}

void KalmanFilter::addHypothesis(const Eigen::Vector3d& pose, double decay_time, double decay_per_update)
{
    Eigen::VectorXd x(6);
    x << pose, 0, 0, 0;

    Eigen::MatrixXd P(6,6);
    P.setIdentity();
    P = P * 2;

    Hypothesis* ptr(new Hypothesis(x, P, decay_time, decay_per_update));
    hypotheses_.push_back(ptr);
}

void KalmanFilter::predict(double dt)
{
    for(std::vector<Hypothesis*>::iterator
            it = hypotheses_.begin();
            it != hypotheses_.end();) {
        if(!(*it)->predict(dt)) {
            INFO("erase " << (*it)->getId());
            delete *it;
            it = hypotheses_.erase(it);
        } else {
            ++it;
        }
    }
}

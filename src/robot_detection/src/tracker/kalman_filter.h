#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <vector>
#include <Eigen/Core>
/// FORWARD DECLARATION
class Hypothesis;

/**
 * @brief The Filter class manages multiple hypotheses
 */
class KalmanFilter
{
public:
    /**
     * @brief Filter
     */
    KalmanFilter();

    /**
     * @brief getBestHypothesis finds the most likely hypothesis
     * @param pose measurement to look for
     * @return the best hypothesis
     */
    Hypothesis* getBestHypothesis(const Eigen::Vector3d& pose);

    /**
     * @brief addHypothesis adds a new hypothesis
     * @param pose the pose to add
     * @param decay_time
     * @param decay_per_update
     */
    void addHypothesis(const Eigen::Vector3d& pose, double decay_time, double decay_per_update);

    /**
     * @brief predict predicts every hypothesis for one timestep
     * @param dt timestep
     */
    void predict(double dt);

    /**
     * @brief count
     * @return the number of hypotheses
     */
    int count() {
        return hypotheses_.size();
    }

    /**
     * @brief get Accessor
     * @param i
     * @return the i-th hypothesis
     */
    Hypothesis* get(int i) {
        return hypotheses_[i];
    }

private:
    std::vector<Hypothesis*> hypotheses_;
};

#endif // KALMAN_FILTER_H

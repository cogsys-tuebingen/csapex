#ifndef TRACKER_H
#define TRACKER_H

/// COMPONENT
#include "kalman_filter.h"
#include "hypothesis.h"

/// SYSTEM
#include <boost/function.hpp>

/// FORWARD DECLARATION
class Pose;

/**
 * @brief The Tracker class manages a filter to track poses
 */
class Tracker
{
public:
    typedef boost::function<void(const Hypothesis& hypo, bool is_valid)> HypothesisIterator;

public:
    /**
     * @brief Tracker
     */
    Tracker();

    /**
     * @brief update Updates the underlying filter with a new measurement
     * @param pose
     */
    void update(const Pose& pose);

    /**
     * @brief predict advances the state of the filter
     * @param dt the time in seconds that has passed since the last call
     */
    void predict(double dt);

    /**
     * @brief forEachHypothesis iterates over all existing hypothesis
     * @param iterator
     */
    void forEachHypothesis(HypothesisIterator iterator);

private:
    KalmanFilter filter;

    double decay_time;
    double decay_per_update;

    double dt_sum;
};

#endif // TRACKER_H

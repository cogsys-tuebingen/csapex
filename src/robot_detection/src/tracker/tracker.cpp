/// HEADER
#include "tracker.h"

/// PROJECT
#include <data/pose.h>
#include <data/angle.h>

Tracker::Tracker()
    : dt_sum(0)
{
    decay_time = 3;
    decay_per_update = 1.5;
}

void Tracker::update(const Pose& pose)
{
    double roll, pitch, yaw;
    Angle::quatToRPY(pose.orientation, roll, pitch, yaw);

    Eigen::Vector3d tracker_pose;
    tracker_pose << pose.position.x(), pose.position.y(), yaw;

    Hypothesis* best = filter.getBestHypothesis(tracker_pose);
    if(best == Hypothesis::NULL_HYPOTHESIS) {
        INFO("add new hypothesis");

        filter.addHypothesis(tracker_pose, decay_time, decay_time);

    } else {
        INFO("update hypothesis");
        best->update(tracker_pose);
    }
}

void Tracker::predict(double dt)
{
    dt_sum += dt;

    if(dt_sum < 0.25) {
        return;
    }

    filter.predict(dt_sum);
    dt_sum = 0;
}

void Tracker::forEachHypothesis(HypothesisIterator iterator)
{
    for(int i = 0, n = filter.count(); i < n; ++i) {
        Hypothesis* hypo = filter.get(i);

        bool is_valid = (hypo->getDecay() > 0 && hypo->getNoTimesUpdated() > 1);

        iterator(*hypo, is_valid);
    }
}

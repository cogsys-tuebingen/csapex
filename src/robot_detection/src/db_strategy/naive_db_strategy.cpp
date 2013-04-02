/// HEADER
#include "naive_db_strategy.h"

/// PROJECT
#include <db/naive_database.h>
#include <data/matchable_pose.h>
#include <utils/matcher.h>

/// SYSTEM
#include <Eigen/Core>

NaiveDatabaseStrategy::NaiveDatabaseStrategy()
    : DatabaseStrategy(new NaiveDatabase)
{
}

NaiveDatabaseStrategy::~NaiveDatabaseStrategy()
{
}

void NaiveDatabaseStrategy::train(Frame::Ptr frame)
{
    double score = INFINITY;
    db->getBestMatch(frame.get(), &score);

    bool enough_points = (int) frame->keypoints.size() > config.min_points;
    INFO("score=" << score);
    if(score > config.score_threshold && enough_points) {
        addFrame(frame);
    }
}

void NaiveDatabaseStrategy::addValidationExample(Frame::Ptr frame)
{
    // nothing to do
}

void NaiveDatabaseStrategy::validate()
{
    // nothing to do, already fully trained at this point
}

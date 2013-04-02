/// HEADER
#include "bow_db_strategy.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <db/bow_database.h>
#include <utils/matcher.h>

BowDatabaseStrategy::BowDatabaseStrategy()
    : DatabaseStrategy(new BowDatabase)
{
    bow_db = dynamic_cast<BowDatabase*>(db);
}

BowDatabaseStrategy::~BowDatabaseStrategy()
{
}

void BowDatabaseStrategy::train(Frame::Ptr frame)
{
    if(!frame->isValid()) {
        WARN("illegal frame, aborting");
        return;
    }

    if((int) frame->keypoints.size() > config.min_points) {
        addFrame(frame);
    }
}

void BowDatabaseStrategy::addValidationExample(Frame::Ptr frame)
{
}

void BowDatabaseStrategy::validate()
{
    db->finishTraining();
}

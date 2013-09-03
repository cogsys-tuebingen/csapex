/// HEADER
#include "bag_db_strategy.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <db/bag_database.h>
#include <utils/matcher.h>

BagDatabaseStrategy::BagDatabaseStrategy()
    : DatabaseStrategy(new BagDatabase)
{
    bag_db = dynamic_cast<BagDatabase*>(db);
}

BagDatabaseStrategy::~BagDatabaseStrategy()
{
}

void BagDatabaseStrategy::train(Frame::Ptr frame)
{
    if(!frame->isValid()) {
        WARN("illegal frame, aborting");
        return;
    }

    if((int) frame->keypoints.size() > config("min_points").as<int>()) {
        addFrame(frame);
    }
}

void BagDatabaseStrategy::addValidationExample(Frame::Ptr /*frame*/)
{
}

void BagDatabaseStrategy::validate()
{
    db->finishTraining();
}

/// HEADER
#include "bin_db_strategy.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <db/bin_database.h>
#include <utils/matcher.h>

BinDatabaseStrategy::BinDatabaseStrategy()
    : DatabaseStrategy(new BinDatabase)
{
    bin_db = dynamic_cast<BinDatabase*>(db);
}

BinDatabaseStrategy::~BinDatabaseStrategy()
{
}

void BinDatabaseStrategy::train(Frame::Ptr frame)
{
    if(!frame->isValid()) {
        WARN("illegal frame, aborting");
        return;
    }

    if((int) frame->keypoints.size() > config("min_points").as<int>()) {
        addFrame(frame);
    }
}

void BinDatabaseStrategy::addValidationExample(Frame::Ptr frame)
{
    bin_db->addValidationExample(frame.get());
}

void BinDatabaseStrategy::validate()
{
    if(config("use_pruning")) {
        bin_db->finishTraining();
    }
}

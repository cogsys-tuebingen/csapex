/// HEADER
#include "bow_database.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <utils/match_scorer.h>

BowDatabase::BowDatabase()
    : trainer(32)
{

}

BowDatabase::~BowDatabase()
{
}

MatchablePose* BowDatabase::getPoseByAngle(const double /*yaw*/, int* /*index*/) const
{
    throw;
}

double BowDatabase::findBestMatch(Matchable* /*current_frame*/, MatchablePose *&/*out*/, int* /*no_of_features*/) const
{
    throw;
}

void BowDatabase::add(MatchablePose* pose)
{
    trainer.add(pose->descriptors);
}

void BowDatabase::finishTraining()
{
    trainer.cluster();
}

void BowDatabase::clear()
{
    trainer.clear();
}

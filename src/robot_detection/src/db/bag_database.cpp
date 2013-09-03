/// HEADER
#include "bag_database.h"

/// PROJECT
#include <utils/match_scorer.h>

BagDatabase::BagDatabase()
    : bag(NULL)
{
    bag = Bag::create();
}

BagDatabase::~BagDatabase()
{
    if(bag) {
        delete bag;
    }
}

void BagDatabase::configChanged()
{
    Database::configChanged();

    if(bag) {
        delete bag;
    }

    bag = Bag::create();
}

MatchablePose* BagDatabase::getPoseByAngle(const double /*yaw*/, int* /*index*/) const
{
    throw;
}

double BagDatabase::findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features) const
{
    double score = bag->query(current_frame, out);

    double match_score = tools->getMatchScorer()->calculateScore(*current_frame, *out, no_of_features);

    INFO("score is: " << score << ", scorer said: " << match_score);

//    return match_score;
    return score;
}

void BagDatabase::add(MatchablePose* pose)
{
    assert(bag);

    bag->add(pose);
}

void BagDatabase::finishTraining()
{
    bag->train();
}

void BagDatabase::clear()
{
    bag->clear();
}

/// HEADER
#include "database.h"

/// PROJECT
#include <data/frame.h>
#include <data/matchable_pose.h>
#include <config/config.h>
#include <utils/extractor.h>
#include <utils/matcher.h>
#include <utils/match_scorer.h>

Database::Database()
    : threshold_score(0)
{
}

Database::~Database()
{
}

MatchablePose* Database::getBestMatch(Matchable* current_frame, double* score_out, int* no_of_features) const
{
    if(score_out) {
        *score_out = INFINITY;
    }

    if((unsigned) current_frame->descriptors.rows < 3) {
        return MatchablePose::NULL_POSE;
    }

    MatchablePose* best_pose = MatchablePose::NULL_POSE;
    double best_score = findBestMatch(current_frame, best_pose, no_of_features);
    assert(best_score >= 0.0);

    if(score_out) {
        *score_out = best_score;
    }

    bool too_far = false;//best_score > threshold_score;

    if(!best_pose || too_far) {
        return MatchablePose::NULL_POSE;
    }

    return best_pose;
}

void Database::traversePoses(boost::function<void (int, const std::string&)> composite_callback, boost::function<void (int, MatchablePose*)> leaf_callback)
{
    ERROR("not implemented");
    throw;
}

/// HEADER
#include "match_scorer_factory.h"

/// COMPONENT
#include "match_scorer_homography.h"
#include "match_scorer_clustering.h"
#include "match_scorer_reprojection.h"

/// PROJECT
#include <common/global.hpp>

MatchScorerFactory::Scorer::Type MatchScorerFactory::create_type = MatchScorerFactory::Scorer::CLUSTERING;

MatchScorerFactory::MatchScorerFactory()
{
}

MatchScorer::Ptr MatchScorerFactory::create(Matcher& matcher)
{
    switch(create_type) {
    case Scorer::HOMOGRAPHY:
        return MatchScorer::Ptr(new MatchScorerHomography(matcher));

    case Scorer::CLUSTERING:
        return MatchScorer::Ptr(new MatchScorerClustering(matcher));

    case Scorer::REPROJECTION:
        return MatchScorer::Ptr(new MatchScorerReprojection(matcher));

    default:
        ERROR("unknown scorer type requested");
        throw IllegalScorerException();
    }

}

void MatchScorerFactory::setType(Scorer::Type type)
{
    MatchScorerFactory::create_type = type;
}

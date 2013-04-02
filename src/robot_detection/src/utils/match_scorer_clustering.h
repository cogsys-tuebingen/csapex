#ifndef MATCH_SCORER_CLUSTERING_H
#define MATCH_SCORER_CLUSTERING_H

/// COMPONENT
#include "match_scorer.h"

/**
 * @brief The MatchScorerClustering class calculates a score based on clustering
 */
class MatchScorerClustering : public MatchScorer
{
public:
    /**
     * @brief MatchScorerClustering
     * @param matcher
     */
    MatchScorerClustering(Matcher& matcher);

    /**
     * @brief calculateScore Tests two Matchables and returns the score
     * @param frame First Matchable to test
     * @param reference Second test object
     * @param no_of_features Output: the number of features found
     * @return the score of the two input objects
     */
    double calculateScore(Matchable& frame, Matchable& reference, int* no_of_features = 0) const;
};

#endif // MATCH_SCORER_CLUSTERING_H

#ifndef MATCH_SCORER_REPROJECTION_H
#define MATCH_SCORER_REPROJECTION_H

/// COMPONENT
#include "match_scorer.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

/**
 * @brief The MatchScorerReprojection class uses the reprojection error to calculate the score
 */
class MatchScorerReprojection : public MatchScorer
{
public:
    /**
     * @brief MatchScorerReprojection
     * @param matcher
     */
    MatchScorerReprojection(Matcher& matcher);

    /**
     * @brief calculateScore Tests two Matchables and returns the score
     * @param frame First Matchable to test
     * @param reference Second test object
     * @param no_of_features Output: the number of features found
     * @return the score of the two input objects
     */
    double calculateScore(Matchable& frame, Matchable& reference, int* no_of_features = NULL) const;
};

#endif // MATCH_SCORER_REPROJECTION_H

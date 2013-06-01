#ifndef MATCH_SCORER_H
#define MATCH_SCORER_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

/// FORWARD DECLARATIONS
class Matchable;
class MatchablePose;
class Matcher;

/**
 * @brief The MatchScorer class is responsible to score the match of two matchable objects
 */
class MatchScorer
{
public:
    typedef boost::shared_ptr<MatchScorer> Ptr;

public:
    /**
     * @brief MatchScorer
     * @param matcher the matcher to use
     */
    MatchScorer(Matcher& matcher);

    /**
     * @brief ~MatchScorer
     */
    virtual ~MatchScorer();

    /**
     * @brief calculateScore Tests two Matchables and returns the score
     * @param frame First Matchable to test
     * @param reference Second test object
     * @param no_of_features Output: the number of features found
     * @return the score of the two input objects
     */
    virtual double calculateScore(Matchable& frame, Matchable& reference, int* no_of_features = 0) const = 0;

protected:
    Matcher& matcher;

public:
    mutable cv::Mat debug;
};

#endif // MATCH_SCORER_H

#ifndef MATCH_SCORER_HOMOGRAPHY_H
#define MATCH_SCORER_HOMOGRAPHY_H

/// COMPONENT
#include "match_scorer.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

/**
 * @brief The MatchScorerHomography class uses a homography to calculate the score
 */
class MatchScorerHomography : public MatchScorer
{
public:
    /**
     * @brief MatchScorerHomography
     * @param matcher
     */
    MatchScorerHomography(Matcher& matcher);

    /**
     * @brief calculateScore Tests two Matchables and returns the score
     * @param frame First Matchable to test
     * @param reference Second test object
     * @param no_of_features Output: the number of features found
     * @return the score of the two input objects
     */
    double calculateScore(Matchable& frame, Matchable& reference, int* no_of_features = NULL) const;

private:
    static double qualityOf(const cv::Mat& H, const cv::Mat& image = cv::Mat());

    static std::vector<cv::Point2f> apply_homography(const cv::Mat& H, const cv::Rect& roi, int padding,
            std::vector<cv::Point2f> *obj_corners = NULL);
    static std::vector<cv::Point2f> apply_homography(const cv::Mat& H, std::vector<cv::Point2f> *obj_corners = NULL);
};

#endif // MATCH_SCORER_HOMOGRAPHY_H

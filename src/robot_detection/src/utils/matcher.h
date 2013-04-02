#ifndef MATCHER_H
#define MATCHER_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

/// FORWARD DECLARATIONS
class Matchable;
class MatchablePose;

/**
 * @brief The Matcher class is responsible for matching descriptors
 */
class Matcher
{
public:
    typedef boost::shared_ptr<Matcher> Ptr;

public:
    /**
     * @brief Matcher
     */
    Matcher(bool binary);

    /**
     * @brief match find pairs of matching keypoints in a and b
     * @param a A matchable object
     * @param b A matchable object
     * @param out Matches between a and b
     */
    void match(const Matchable* a, const Matchable* b, std::vector<std::vector<cv::DMatch> > &out) const;

    /**
     * @brief matchFiltered like 'match', but filters matched keypoints into vectors
     * @param a A matchable object
     * @param b A matchable object
     * @param filtered_a_keypoints Every keypoint in 'a' that found a partner in 'b'
     * @param filtered_b_keypoints Every keypoint in 'b' that found a partner in 'a'
     * @return The ratio of |a| / |filtered_a|
     */
    double matchFiltered(Matchable* a, const Matchable* b,
                         std::vector<cv::KeyPoint> &filtered_a_keypoints,
                         std::vector<cv::KeyPoint> &filtered_b_keypoints,
                         std::vector<std::vector<cv::DMatch> > * match_out = NULL) const;

    /**
     * @brief matchAntiFilter Removes every keypoint (incl. descriptor) of 'positive' that doesn't match any keypoint of 'negative'
     * @param positive Matchable to be filtered
     * @param negative Negative example
     */
    void matchAntiFilter(Matchable* positive, const Matchable* negative) const;

    /**
     * @brief getMinPointCount
     * @return the minimum numbers of keypoints that must be present for a sucessful match
     */
    int getMinPointCount() {
        return std::max(4, min_points);
    }

protected:
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;

    bool hamming;

    double threshold;

    int min_points;
};

#endif // MATCHER_H

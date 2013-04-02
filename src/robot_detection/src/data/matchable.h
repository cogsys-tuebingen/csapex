#ifndef MATCHABLE_H
#define MATCHABLE_H

/// COMPONENT
#include "angle.h"

/// SYSTEM
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4
#include <opencv2/nonfree/nonfree.hpp>
#endif

/**
 * @brief The Matchable class is a base class for everything that has keypoints and descriptors
 */
class Matchable
{
protected:
    /**
     * @brief Matchable
     * @param keypoints
     * @param descriptors
     */
    Matchable(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat& descriptors);

    /**
     * @brief Matchable empty constructor
     */
    Matchable();

public:
    /**
     * @brief ~Matchable
     */
    virtual ~Matchable();

    /**
     * @brief get_dimensions
     * @return the width and height of this matchable
     */
    virtual cv::Rect getDimensions() const = 0;

public:
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    std::vector<std::vector<cv::DMatch> > last_matches;
    cv::Rect last_roi;

    Angle orientation;
    double distance;
};

#endif // MATCHABLE_H

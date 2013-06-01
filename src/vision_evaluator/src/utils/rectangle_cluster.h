#ifndef RECTANGLE_CLUSTER_H
#define RECTANGLE_CLUSTER_H

/// SYSTEM
#include <opencv2/opencv.hpp>

/**
 * @brief The RectangleCluster class represents a collection of rectangles.
 *        Everytime a new rectangle is added, it is merged with the existing ones.
 *        At no time are there any two rectangles that intersect each other.
 */
class RectangleCluster
{
public:
    /**
     * @brief RectangleCluster
     */
    RectangleCluster();

    /**
     * @brief integrate add another rectangle
     * @param rectangle
     */
    void integrate(const cv::Rect& rectangle);

    /**
     * @brief begin
     * @return iterator to the beginning
     */
    std::vector<cv::Rect>::iterator begin() {
        return rois.begin();
    }

    /**
     * @brief begin
     * @return iterator to the end
     */
    std::vector<cv::Rect>::iterator end() {
        return rois.end();
    }

    /**
     * @brief conflicts tests of the given rectangle intersects any integrated rectangle
     * @param test
     * @return true, iff there is at least one rectangle that intersects "test"
     */
    bool conflicts(const cv::Rect& test);

    /**
     * @brief contains tests of the given rectangle equals any integrated rectangle
     * @param test
     * @return true, iff there is at least one rectangle that equals "test"
     */
    bool contains(const cv::Rect& test);

    /**
     * @brief covers tests of the given rectangle is fully contained in any integrated rectangle
     * @param test
     * @return true, iff there is at least one rectangle fully contains "test"
     */
    bool covers(const cv::Rect& test);

private:
    std::vector<cv::Rect> rois;
};

#endif // RECTANGLE_CLUSTER_H

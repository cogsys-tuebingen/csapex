#ifndef DEBUG_PAINTER_H
#define DEBUG_PAINTER_H

/// PROJECT
#include <data/frame.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

/// FORWARD DECLARATION
class Angle;
class Matchable;

class Painter
{
public:
    /**
     * @brief Painter
     * @param target
     */
    Painter(Frame::Ptr target);

    /**
     * @brief Painter
     * @param target
     */
    Painter(Matchable* target);

    /**
     * @brief canPaint
     * @return  true, iff target is paintable
     */
    bool canPaint();

    /**
     * @brief displayImage
     * @param name title of the window
     */
    void displayImage(std::string name = "debug");

    /**
     * @brief visualizeOrientation
     * @param yaw Angle
     * @param target Output: image
     * @param center the point to draw away from
     * @param color the color to use
     */
    void visualizeOrientation(const Angle& yaw, cv::Mat* target = NULL, const cv::Point& center = cv::Point(), const cv::Scalar& color = cv::Scalar(255, 0, 0));

    /**
     * @brief drawLineRoi draw a line
     * @param from
     * @param to
     * @param color
     * @param width line width
     * @param target Output: image
     */
    void drawLineRoi(const cv::Point& from, const cv::Point& to, cv::Scalar color, int width = 1, cv::Mat* target = NULL);

    /**
     * @brief drawRectangleRoi draw a rectangle
     * @param rect
     * @param color
     * @param width
     */
    void drawRectangleRoi(const cv::Rect& rect, cv::Scalar color = cv::Scalar(255, 0, 0), int width = 2);

    /**
     * @brief drawCircleRoi draw a circle
     * @param center
     * @param radius
     * @param color
     */
    void drawCircleRoi(const cv::Point& center, float radius, cv::Scalar color = cv::Scalar(255, 0, 0));

    /**
     * @brief drawRectangle draw a rectangle
     * @param rect
     * @param color
     * @param width
     */
    void drawRectangle(const cv::Rect& rect, cv::Scalar color = cv::Scalar(255, 0, 0), int width = 2);

    /**
     * @brief drawKeypointsRoi visualize keypoints
     * @param color color to use
     * @param flags cv::DrawMatchesFlags
     */
    void drawKeypointsRoi(cv::Scalar color, int flags = cv::DrawMatchesFlags::DEFAULT);

    /**
     * @brief drawKeypointsRoiHighlight visualize keypoints user provided keypoints
     * @param pts points to visualize
     * @param color color to use
     * @param flags cv::DrawMatchesFlags
     */
    void drawKeypointsRoiHighlight(std::vector<cv::KeyPoint> &pts, cv::Scalar color, int flags = cv::DrawMatchesFlags::DEFAULT);

    /**
     * @brief drawKeypointsRoiOverlay visualize keypoints transparently
     * @param color color to use
     * @param flags cv::DrawMatchesFlags
     */
    void drawKeypointsRoiOverlay(cv::Scalar color, int flags = cv::DrawMatchesFlags::DEFAULT);

    /**
     * @brief drawRoi draws a rectangle around the current region of interest
     * @param color border color to use
     */
    void drawRoi(cv::Scalar color = cv::Scalar(255, 0, 0));

private:
    Frame* frame;
};

#endif // DEBUG_PAINTER_H

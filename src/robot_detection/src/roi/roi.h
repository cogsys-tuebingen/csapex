#ifndef ROI_H
#define ROI_H

/// SYSTEM
#include <opencv2/core/core.hpp>

class Roi
{
public:
    Roi();
    Roi(cv::Rect rectangle, cv::Scalar color = cv::Scalar(255, 255, 0));
    Roi(int x, int y, int width, int height, cv::Scalar color = cv::Scalar(255, 255, 0));

    bool covers(const Roi& rhs) const;

    void grow(int pixels_x, int pixels_y);

private:
    void check();

public:
    cv::Rect rect;
    cv::Scalar color;
};

#endif // ROI_H

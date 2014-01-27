#ifndef ROI_H
#define ROI_H

/// SYSTEM
#include <opencv2/core/core.hpp>

namespace csapex
{

class Roi
{
public:
    Roi();
    Roi(cv::Rect rectangle, cv::Scalar color = cv::Scalar(255, 255, 0));
    Roi(int x, int y, int width, int height, cv::Scalar color = cv::Scalar(255, 255, 0));

    bool covers(const Roi& rhs) const;

    void grow(int pixels_x, int pixels_y);

    int x() const;
    int y() const;
    int w() const;
    int h() const;

    void setLabel(const std::string& label);
    std::string label() const;

    cv::Rect rect() const;
    cv::Scalar color() const;

private:
    void check();

private:
    cv::Rect rect_;
    cv::Scalar color_;
    std::string label_;
};

}

#endif // ROI_H

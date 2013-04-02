/// HEADER
#include "roi.h"

Roi::Roi()
{

}

Roi::Roi(cv::Rect rectangle, cv::Scalar color)
    : rect(rectangle), color(color)
{
    check();
}

Roi::Roi(int x, int y, int width, int height, cv::Scalar color)
    : rect(x, y, width, height), color(color)
{
    check();
}

void Roi::check()
{
    assert(rect.x >= 0);
    assert(rect.y >= 0);
}

bool Roi::covers(const Roi& rhs) const
{
    if(rect == rhs.rect || rect == (rect | rhs.rect)) {
        /// rhs is equal to roi
        ///  or contained in roi
        return true;
    }

    return false;
}

void Roi::grow(int pixels_x, int pixels_y)
{
    rect = rect - cv::Point(pixels_x / 2, pixels_y / 2) + cv::Size(pixels_x, pixels_y);
}

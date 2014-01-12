/// HEADER
#include <csapex_vision/roi.h>

using namespace csapex;

Roi::Roi()
{

}

Roi::Roi(cv::Rect rectangle, cv::Scalar color)
    : rect_(rectangle), color_(color)
{
    check();
}

Roi::Roi(int x, int y, int width, int height, cv::Scalar color)
    : rect_(x, y, width, height), color_(color)
{
    check();
}

void Roi::check()
{
    assert(rect_.x >= 0);
    assert(rect_.y >= 0);
}

bool Roi::covers(const Roi& rhs) const
{
    if(rect_ == rhs.rect_ || rect_ == (rect_ | rhs.rect_)) {
        /// rhs is equal to roi
        ///  or contained in roi
        return true;
    }

    return false;
}

void Roi::grow(int pixels_x, int pixels_y)
{
    rect_ = rect_ - cv::Point(pixels_x / 2, pixels_y / 2) + cv::Size(pixels_x, pixels_y);
}

int Roi::x() const
{
    return rect_.x;
}


int Roi::y() const
{
    return rect_.y;
}


int Roi::w() const
{
    return rect_.width;
}


int Roi::h() const
{
    return rect_.height;
}

cv::Rect Roi::rect() const
{
    return rect_;
}

cv::Scalar Roi::color() const
{
    return color_;
}

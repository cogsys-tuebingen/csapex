/*
 * opencv_utils.hpp
 *
 *  Created on: Mar 8, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef OPENCV_UTILS_HPP
#define OPENCV_UTILS_HPP

namespace cv
{
const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;

static inline void drawKeypoint(Mat& img, const KeyPoint& p, const Scalar& color, int flags)
{
    CV_Assert(!img.empty());
    Point center(cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier));

    if(flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS) {
        int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter

        // draw the circles around keypoints with the keypoints size
        circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);

        // draw orientation of the keypoint, if it is applicable
        if(p.angle != -1) {
            float srcAngleRad = p.angle*(float)CV_PI/180.f;
            Point orient(cvRound(cos(srcAngleRad)*radius),
                         cvRound(sin(srcAngleRad)*radius)
                        );
            line(img, center, center+orient, color, 1, CV_AA, draw_shift_bits);
        }
#if 0
        else {
            // draw center with R=1
            int radius = 1 * draw_multiplier;
            circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
        }
#endif
    } else {
        // draw center with R=3
        int radius = 3 * draw_multiplier;
        circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
    }
}

static inline void drawConvexHull(const std::vector<Point>& hull, Mat& img, const Scalar& color)
{
    const Point* last = &hull[hull.size()-1];
    for(unsigned j = 0; j < hull.size(); ++j) {
        line(img, *last, hull[j], color, 1, CV_AA);
        last = &hull[j];
    }
}

static cv::Scalar rangeColor(int i, int k)
{
    cv::Mat col(1,1,CV_8UC3, cv::Scalar((255.0 * i) / k, 255, 255));
    cv::cvtColor(col, col, CV_HSV2BGR);
    cv::Vec3b s = col.at<cv::Vec3b>(0,0);
    return cv::Scalar(s[0], s[1], s[2]);
}
}

#endif // OPENCV_UTILS_HPP

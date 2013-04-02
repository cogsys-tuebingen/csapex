/// HEADER
#include "painter.h"

/// COMPONENT
#include "frame.h"

Painter::Painter(Matchable* target)
    : frame(dynamic_cast<Frame*>(target))
{
}

Painter::Painter(Frame::Ptr target)
    : frame(target.get())
{
}

bool Painter::canPaint()
{
    return frame != NULL;
}

void Painter::displayImage(std::string name)
{
    cv::imshow(name, frame->image_raw);
    cv::waitKey(33);
}

void Painter::visualizeOrientation(const Angle& yaw, cv::Mat* target, const cv::Point& center, const cv::Scalar& color)
{
    if(!frame) return;

    Eigen::Quaterniond orientation = yaw.toQuaternion();

    /*   z   x
     *    | /
     * ___|/
     * y
     */
    Eigen::Vector3d line_direction(1, 0, 0);

    line_direction = orientation * line_direction;


    /*    z = x'
     *    /
     *   /___x  = -y'
     *   |
     *   | y = -z'
     */

    Eigen::Matrix3d base_change;
    base_change << 0,  0,  1,
                -1,  0,  0,
                0, -1,  0;
    Eigen::AngleAxisd tilt = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY());

    line_direction = tilt * (base_change * line_direction);

    Eigen::Vector3d projection2d = line_direction * 100.0;
    projection2d(2) = 0;

    cv::Point c;
    if(center.x == 0 && center.y == 0) {
        c = cv::Point(frame->getRoiWidth()/2.0,  frame->getRoiHeight()/2.0);
    } else {
        c = center;
    }
    cv::Point to(c.x + projection2d.x(), c.y + projection2d.y());

    drawCircleRoi(c, 10, color);
    drawLineRoi(c, to, color, 4, target);
}


void Painter::drawLineRoi(const cv::Point& from, const cv::Point& to, cv::Scalar color, int width, cv::Mat* target)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    if(target == NULL) {
        // manually draw into roi to that lines can overlap to borders
        cv::Point from_roi(from.x + frame->roi.x, from.y + frame->roi.y);
        cv::Point to_roi(to.x + frame->roi.x, to.y + frame->roi.y);
        cv::line(frame->debug_img_tmp, from_roi, to_roi, color, width, CV_AA);

    } else {
        cv::line(*target, from, to, color, width, CV_AA);
    }
}


void Painter::drawRoi(cv::Scalar color)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::rectangle(frame->debug_img_tmp, frame->roi, color, 1, CV_AA);
}

void Painter::drawRectangle(const cv::Rect& rect, cv::Scalar color, int width)
{
    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::rectangle(frame->debug_img_tmp, rect, color, width, CV_AA);
}

void Painter::drawRectangleRoi(const cv::Rect& rect, cv::Scalar color, int width)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::rectangle(frame->debug_img_tmp_roi, rect, color, width, CV_AA);
}

void Painter::drawCircleRoi(const cv::Point& center, float radius, cv::Scalar color)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::circle(frame->debug_img_tmp_roi, center, radius, color, CV_AA);
}

void Painter::drawKeypointsRoi(cv::Scalar color, int flags)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::drawKeypoints(frame->debug_img_tmp_roi, frame->keypoints, frame->debug_img_tmp_roi, color);
}

void Painter::drawKeypointsRoiOverlay(cv::Scalar color, int flags)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::Mat tmp(frame->debug_img_tmp_roi.rows, frame->debug_img_tmp_roi.cols, frame->debug_img_tmp_roi.type(), cv::Scalar(0));
    cv::drawKeypoints(tmp, frame->keypoints, tmp, color, flags);
    cv::addWeighted(frame->debug_img_tmp_roi, 1.0, tmp, 0.7, 0, frame->debug_img_tmp_roi);
}

void Painter::drawKeypointsRoiHighlight(std::vector<cv::KeyPoint> &pts, cv::Scalar color, int flags)
{
    if(!frame) return;

    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::drawKeypoints(frame->debug_img_tmp_roi, pts, frame->debug_img_tmp_roi, color, flags);
}

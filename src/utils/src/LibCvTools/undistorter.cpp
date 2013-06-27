#include "undistorter.h"

Undistorter::Undistorter(const cv::Mat &camera_mat, const cv::Mat &distoration_coeffs, const int interpolation) :
    interpolation_method_(interpolation),
    camera_mat_(camera_mat),
    distortion_coeffs_(distoration_coeffs)
{
}

Undistorter::~Undistorter()
{
}

void Undistorter::undistort_nomap(const cv::Mat &src, cv::Mat &dst)
{
    cv::undistort(src, dst, camera_mat_, distortion_coeffs_);
}

void Undistorter::undistort(const cv::Mat &src, cv::Mat &dst)
{
    cv::remap(src, dst, undist_map_x_, undist_map_y_, interpolation_method_);
}

void Undistorter::undistort_points_nomap(const cv::Mat &src, cv::Mat &dst)
{
    cv::undistortPoints(src, dst, camera_mat_, distortion_coeffs_);
}

void Undistorter::undistort_points_nomap(const std::vector<cv::Point2d> &src, std::vector<cv::Point2d> &dst)
{
    cv::undistortPoints(src, dst, camera_mat_, distortion_coeffs_);
}

void Undistorter::undistort_points_nomap(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst)
{
    cv::undistortPoints(src, dst, camera_mat_, distortion_coeffs_);
}

void Undistorter::reset_map(const cv::Size s, const float camera_offset_x, const float camera_offset_y)
{
    if(undist_img_size_ != s) {
        cv::Mat tmp_cam = camera_mat_.clone();

        tmp_cam.at<float>(0,2) += camera_offset_x;
        tmp_cam.at<float>(1,2) += camera_offset_y;

        cv::initUndistortRectifyMap(tmp_cam, distortion_coeffs_,
                                    orientation_, optimal_camera_mat_, s,
                                    CV_32FC1, undist_map_x_, undist_map_y_);
        undist_img_size_ = s;
    }
}




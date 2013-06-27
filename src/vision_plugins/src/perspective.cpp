#include "perspective.h"

static double _PI_180(M_PI / 180.0);
static double _90_RAD(90 * _PI_180);

PerspectiveTransformer::PerspectiveTransformer() :
    img_size_(0, 0)
{
    init();
}

PerspectiveTransformer::~PerspectiveTransformer()
{
}

void PerspectiveTransformer::init()
{
    proj_to2d_ = ( cv::Mat_<double>(3,4) << 500, 0, 0, 0,
                                            0, 500, 0, 0,
                                            0, 0, 1, 0);

    proj_to3d_ = ( cv::Mat_<double>(4,3) << 1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 0,
                                            0, 0, 1);

    rot_x_   = ( cv::Mat_<double>(4,4) << 1, 0, 0, 0,
                                          0, 1, 0, 0,
                                          0, 0, 1, 0,
                                          0, 0, 0, 1);
    rot_y_      = rot_x_.clone();
    rot_z_      = rot_x_.clone();
    distance_   = rot_x_.clone();
    distance_.at<double>(2,3) = 500;

    dirty_ = true;
}

void PerspectiveTransformer::set_rot_x(const double ang)
{
    set_rot_x_rad(ang * _PI_180);
}

void PerspectiveTransformer::set_rot_y(const double ang)
{
    set_rot_y_rad(ang * _PI_180);
}

void PerspectiveTransformer::set_rot_z(const double ang)
{
    set_rot_z_rad(ang * _PI_180);
}

void PerspectiveTransformer::set_rot_x_rad(const double rad)
{
    rot_x_.at<double>(1,1) =  std::cos(rad);
    rot_x_.at<double>(1,2) = -std::sin(rad);
    rot_x_.at<double>(2,1) =  std::sin(rad);
    rot_x_.at<double>(2,2) =  std::cos(rad);
    dirty_ = true;
}

void PerspectiveTransformer::set_rot_y_rad(const double rad)
{
    rot_y_.at<double>(0,0) =  std::cos(rad);
    rot_y_.at<double>(0,2) = -std::sin(rad);
    rot_y_.at<double>(2,0) =  std::sin(rad);
    rot_y_.at<double>(2,2) =  std::cos(rad);
    dirty_ = true;
}

void PerspectiveTransformer::set_rot_z_rad(const double rad)
{
    rot_y_.at<double>(0,0) =  std::cos(rad);
    rot_y_.at<double>(0,1) = -std::sin(rad);
    rot_y_.at<double>(1,0) =  std::sin(rad);
    rot_y_.at<double>(1,1) =  std::cos(rad);
    dirty_ = true;
}

void PerspectiveTransformer::set_distance(const double dist)
{
    distance_.at<double>(2,3) = dist;
    dirty_ = true;
}

void PerspectiveTransformer::set_focal(const double focal)
{
    proj_to2d_.at<double>(0,0) = focal;
    proj_to2d_.at<double>(1,1) = focal;
    dirty_ = true;
}

void PerspectiveTransformer::transform(const cv::Mat &src, cv::Mat &dst)
{
    if(src.cols != img_size_.width || src.rows != img_size_.height || dirty_) {
        img_size_.width = src.cols;
        img_size_.height = src.rows;
        recompute_transform();
    }

    cv::warpPerspective(src, dst, transformation_ , img_size_, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void PerspectiveTransformer::recompute_transform()
{
    /// prepare projection matrix
    proj_to3d_.at<double>(0,2) = -img_size_.width * 0.5;
    proj_to3d_.at<double>(1,2) = -img_size_.height * 0.5;
    proj_to2d_.at<double>(0,2) =  img_size_.width * 0.5;
    proj_to2d_.at<double>(1,2) =  img_size_.height * 0.5;

    transformation_ = proj_to2d_ * (distance_ * ((rot_x_ * rot_y_ * rot_z_) * proj_to3d_));

     dirty_ = false;
}

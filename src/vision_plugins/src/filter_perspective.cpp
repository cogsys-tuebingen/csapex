/// HEADER
#include "filter_perspective.h"

/// PROJECT
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_transformation::PerspectiveTransform, vision_evaluator::Filter)

using namespace image_transformation;

PerspectiveTransform::PerspectiveTransform()
{
}

void PerspectiveTransform::filter(cv::Mat &img, cv::Mat &mask)
{
    /// mask is unused
    transformer_.transform(img, img);
}

void PerspectiveTransform::insert(QBoxLayout *parent)
{
    rot_x_ = QtHelper::makeSlider(parent, "Rotation x", 0, -90, 90);
    rot_y_ = QtHelper::makeSlider(parent, "Rotation y", 0, -90, 90);
    rot_z_ = QtHelper::makeSlider(parent, "Rotation z", 0, -90, 90);
    distance_     = QtHelper::makeSlider(parent, "Virt. Distance", 500, 0, 2000);
    focal_length_ = QtHelper::makeSlider(parent, "Virt. Focal Length", 500, 0, 2000);

    connect(rot_x_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(rot_y_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(rot_z_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(distance_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(focal_length_, SIGNAL(valueChanged(int)), this, SLOT(update()));

}

void PerspectiveTransform::update()
{
    transformer_.set_rot_x(rot_x_->value());
    transformer_.set_rot_y(rot_y_->value());
    transformer_.set_rot_z(rot_z_->value());
    transformer_.set_distance(distance_->value());
    transformer_.set_focal(focal_length_->value());
}

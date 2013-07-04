/// HEADER
#include "filter_perspective.h"

/// PROJECT
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_plugins::PerspectiveTransform, vision_evaluator::Filter)

using namespace vision_plugins;

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
    rot_x_ = QtHelper::makeDoubleSlider(parent, "Rotation x", 0, -90, 90, 0.1);
    rot_y_ = QtHelper::makeDoubleSlider(parent, "Rotation y", 0, -90, 90, 0.1);
    rot_z_ = QtHelper::makeDoubleSlider(parent, "Rotation z", 0, -90, 90, 0.1);
    distance_     = QtHelper::makeDoubleSlider(parent, "Virt. Distance", 500, 0, 2000, 0.1);
    focal_length_ = QtHelper::makeDoubleSlider(parent, "Virt. Focal Length", 500, 0, 2000, 0.1);

    connect(rot_x_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(rot_y_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(rot_z_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(distance_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(focal_length_, SIGNAL(valueChanged(int)), this, SLOT(update()));

}

void PerspectiveTransform::update()
{
    transformer_.set_rot_x(rot_x_->doubleValue());
    transformer_.set_rot_y(rot_y_->doubleValue());
    transformer_.set_rot_z(rot_z_->doubleValue());
    transformer_.set_distance(distance_->doubleValue());
    transformer_.set_focal(focal_length_->doubleValue());
}

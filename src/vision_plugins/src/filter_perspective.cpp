/// HEADER
#include "filter_perspective.h"

/// PROJECT
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_plugins::PerspectiveTransform, csapex::BoxedObject)

using namespace vision_plugins;
using namespace csapex;

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
    slider_rot_x_ = QtHelper::makeDoubleSlider(parent, "Rotation x", 0, -90, 90, 0.1);
    slider_rot_y_ = QtHelper::makeDoubleSlider(parent, "Rotation y", 0, -90, 90, 0.1);
    slider_rot_z_ = QtHelper::makeDoubleSlider(parent, "Rotation z", 0, -90, 90, 0.1);
    slider_distance_     = QtHelper::makeDoubleSlider(parent, "Virt. Distance", 500, 0, 2000, 0.1);
    slider_focal_length_ = QtHelper::makeDoubleSlider(parent, "Virt. Focal Length", 500, 0, 2000, 0.1);

    connect(slider_rot_x_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(slider_rot_y_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(slider_rot_z_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(slider_distance_, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(slider_focal_length_, SIGNAL(valueChanged(int)), this, SLOT(update()));

}

void PerspectiveTransform::update()
{
    state_.rot_x = slider_rot_x_->doubleValue();
    state_.rot_y = slider_rot_y_->doubleValue();
    state_.rot_z = slider_rot_z_->doubleValue();
    state_.foca  = slider_focal_length_->doubleValue();
    state_.dist  = slider_distance_->doubleValue();
    transformer_.set_rot_x(state_.rot_x);
    transformer_.set_rot_y(state_.rot_y);
    transformer_.set_rot_z(state_.rot_z);
    transformer_.set_distance(state_.dist);
    transformer_.set_focal(state_.foca);
}

Memento::Ptr PerspectiveTransform::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void PerspectiveTransform::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;

    slider_rot_x_->setDoubleValue(state_.rot_x);
    slider_rot_y_->setDoubleValue(state_.rot_y);
    slider_rot_z_->setDoubleValue(state_.rot_z);
    slider_distance_->setDoubleValue(state_.dist);
    slider_focal_length_->setDoubleValue(state_.foca);
}

/// MEMENTO
void PerspectiveTransform::State::readYaml(const YAML::Node &node)
{
    node["rot_x"] >> rot_x;
    node["rot_y"] >> rot_y;
    node["rot_z"] >> rot_z;
    node["foca"] >> foca;
    node["dist"] >> dist;
}

void PerspectiveTransform::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "rot_x" << YAML::Value << rot_x;
    out << YAML::Key << "rot_y" << YAML::Value << rot_y;
    out << YAML::Key << "rot_z" << YAML::Value << rot_z;
    out << YAML::Key << "foca" << YAML::Value << foca;
    out << YAML::Key << "dist" << YAML::Value << dist;
}


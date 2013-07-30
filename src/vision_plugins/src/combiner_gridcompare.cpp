#include "combiner_gridcompare.h"

/// PROJECT
#include <csapex/qt_helper.hpp>

using namespace csapex;

GridCompare::GridCompare(State::Ptr state) :
    state_(state)
{
}

void GridCompare::fill(QBoxLayout *layout)
{
    ImageCombiner::fill(layout);
    addSliders(layout);
}


void GridCompare::addSliders(QBoxLayout *layout)
{
    slide_width_  = QtHelper::makeSlider(layout, "Grid Width",  64, 1, 640);
    slide_height_ = QtHelper::makeSlider(layout, "Grid Height", 48, 1, 640);

}

void GridCompare::updateSliderMaxima(int width, int height)
{
    if(state_->grid_height_max != height) {
        state_->grid_height_max = height;
        slide_height_->setMaximum(height);
    }
    if(state_->grid_width_max != width) {
        state_->grid_width_max = width;
        slide_width_->setMaximum(width);
    }
}

void GridCompare::updateDynamicGui(QBoxLayout *layout)
{
}

/// MEMENTO ------------------------------------------------------------------------------------
GridCompare::State::State() :
    channel_count(0),
    grid_width(64),
    grid_height(48),
    grid_width_max(640),
    grid_height_max(480)
{
}

void GridCompare::State::readYaml(const YAML::Node &node)
{
    node["channel_count"] >> channel_count;
    node["grid_width"] >> grid_width;
    node["grid_height"] >> grid_height;
    node["grid_width_max"] >> grid_width_max;
    node["grid_height_max"] >> grid_height_max;
}

void GridCompare::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "channel_count" << YAML::Value << channel_count;
    out << YAML::Key << "grid_width" << YAML::Value << grid_width;
    out << YAML::Key << "grid_height" << YAML::Value << grid_height;
    out << YAML::Key << "grid_width_max" << YAML::Value << grid_width_max;
    out << YAML::Key << "grid_height_max" << YAML::Value << grid_height_max;

}

/// HEADER
#include "filter_equalize.h"

/// PROJECT
#include <utils/LibCvTools/histogram.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_plugins::Equalize, vision_evaluator::Filter)

using namespace vision_plugins;

Equalize::Equalize()
{
}

Equalize::~Equalize()
{
}

void Equalize::insert(QBoxLayout *parent)
{
}

void Equalize::filter(cv::Mat &img, cv::Mat &mask)
{
    cv_histogram::full_channel_equalize(img, img);
}

bool Equalize::usesMask()
{
    return false;
}

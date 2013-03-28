/// HEADER
#include "plugin_base.h"

/// COMPONENT
#include "plugin_queue.h"

using namespace vision_evaluator;

PluginBase::PluginBase(const std::string& name)
    : name_(name), queue_(NULL)
{
}

PluginBase::~PluginBase()
{
}

std::string PluginBase::getName()
{
    return name_;
}

void PluginBase::setQueue(PluginQueue* queue)
{
    queue_ = queue;
}

PluginBase::PluginPtr PluginBase::metaInstance()
{
    std::cerr << "class " << name_ << " is not a meta plugin" << std::endl;
    throw;
}

void PluginBase::filter(cv::Mat img, cv::Mat mask)
{
}


cv::Mat PluginBase::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    return img1;
}

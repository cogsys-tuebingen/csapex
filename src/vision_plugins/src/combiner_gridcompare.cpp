#include "combiner_gridcompare.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompare, vision_evaluator::ImageCombiner)


using namespace vision_evaluator;

GridCompare::GridCompare()
{
}

cv::Mat GridCompare::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{

    return cv::Mat();
}

void GridCompare::fill(QBoxLayout *layout)
{
    ImageCombiner::fill(layout);
}

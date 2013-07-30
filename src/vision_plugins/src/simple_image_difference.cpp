/// HEADER
#include "simple_image_difference.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

using namespace vision_plugins;

PLUGINLIB_EXPORT_CLASS(vision_plugins::SimpleImageDifference, csapex::BoxedObject);

SimpleImageDifference::SimpleImageDifference()
{
}

cv::Mat SimpleImageDifference::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    cv::Mat out;
    cv::absdiff(img1, img2, out);
    return out;
}

void SimpleImageDifference::insert(QBoxLayout *layout)
{

}

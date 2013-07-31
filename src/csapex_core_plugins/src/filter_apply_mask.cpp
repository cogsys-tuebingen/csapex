/// HEADER
#include "filter_apply_mask.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(csapex::FilterApplyMask, csapex::BoxedObject)

using namespace csapex;

FilterApplyMask::FilterApplyMask()
{
}

void FilterApplyMask::filter(cv::Mat& img, cv::Mat& mask)
{
    if(!mask.empty()) {
        cv::Mat tmp;
        img.copyTo(tmp, mask);
        img = tmp;
    }
}

void FilterApplyMask::insert(QBoxLayout* layout)
{
}

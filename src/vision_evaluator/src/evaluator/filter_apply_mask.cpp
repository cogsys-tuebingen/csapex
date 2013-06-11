/// HEADER
#include "filter_apply_mask.h"

/// COMPONENT
#include "filter_manager.h"

REGISTER_FILTER(FilterApplyMask)

using namespace vision_evaluator;

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

/// HEADER
#include "filter_apply_mask.h"

/// COMPONENT
#include "registration.hpp"

REGISTER_BOXED_OBJECT(FilterApplyMask)

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

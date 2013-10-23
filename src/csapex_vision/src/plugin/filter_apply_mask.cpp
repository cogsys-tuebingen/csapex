/// HEADER
#include "filter_apply_mask.h"

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::FilterApplyMask, csapex::BoxedObject)

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

#include "cmp_core.h"
#include <yaml.h>
#include "extractors.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractor)
{
}

bool CMPCore::load(const std::string image_path)
{
    raw_image_ = cv::imread(image_path);
    return !raw_image_.empty();
}

cv::Mat CMPCore::getImage() const
{
    return raw_image_.clone();
}

void CMPCore::addROI(const ROI &roi)
{

}

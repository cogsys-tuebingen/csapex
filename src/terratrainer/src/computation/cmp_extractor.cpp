#include "cmp_extractor.h"
#include <opencv2/nonfree/features2d.hpp>

CMPExtractor::CMPExtractor()
{
}

void CMPExtractor::set(cv::DescriptorExtractor *extractor)
{
    extractor_.reset(extractor);
}

/// HEADER
#include "matchable.h"

Matchable::Matchable(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat& descriptors)
    : keypoints(keypoints), descriptors(descriptors), distance(-1)
{
}

Matchable::Matchable()
    : distance(-1)
{
}

Matchable::~Matchable()
{
}

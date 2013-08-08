#include "extractor.h"
#include <cmath>

Extractor::Extractor()
{
}

void Extractor::set(cv::DescriptorExtractor *extractor)
{
    extractor_.reset(extractor);
}

void Extractor::extract(const cv::Mat &image, cv::Mat &descriptors)
{
    std::vector<cv::KeyPoint> key_points;
    cv::KeyPoint k(image.cols / 2.0, image.rows / 2.0, image.cols);
    k.octave = 0;
    key_points.push_back(k);
    extract(image, key_points, descriptors);
}

void Extractor::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors)
{
    extractor_->compute(image, key_points, descriptors);
}

Extractor::KeyPoints Extractor::prepareKeypoint(cv::Rect rect)
{
    /// TODO : CHECK THE KEYPOINT PROPERTIES FOR DIFFERENT EXTRACTORS
    KeyPoints key_points;
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height);
    k.octave = 0;
    key_points.push_back(k);
    return key_points;
}


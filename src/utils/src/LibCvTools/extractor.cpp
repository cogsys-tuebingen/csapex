#include "extractor.h"
#include <cmath>

CVExtractor::CVExtractor()
{
}

void CVExtractor::set(cv::DescriptorExtractor *extractor)
{
    extractor_.reset(extractor);
}

void CVExtractor::extract(const cv::Mat &image, cv::Mat &descriptors)
{
    std::vector<cv::KeyPoint> key_points;
    cv::KeyPoint k(image.cols / 2.0, image.rows / 2.0, 5.0);
    k.octave = 0;
    key_points.push_back(k);
    extract(image, key_points, descriptors);
}

void CVExtractor::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors)
{
    extractor_->compute(image, key_points, descriptors);
}

CVExtractor::KeyPoints CVExtractor::prepareKeypoint(const cv::Rect &rect, const bool soft_crop, const float scale, const float angle)
{
    /// TODO : CHECK THE KEYPOINT PROPERTIES FOR DIFFERENT EXTRACTORS
    KeyPoints key_points;
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height / 2.0 * scale, angle);
    if(soft_crop) {
        k.pt.x += rect.x;
        k.pt.y += rect.y;
    }

    key_points.push_back(k);
    return key_points;
}

CVExtractor::KeyPoints CVExtractor::prepareNeighbouredKeypoint(const Rect &rect, const bool soft_crop, const float scale, const float angle)
{
    KeyPoints key_points;
    double scaled = rect.height / 2.0 * scale;
    cv::KeyPoint center(rect.width / 2.0, rect.height / 2.0, scaled, angle);
    cv::KeyPoint leftUp(0, 0, scaled, angle);
    cv::KeyPoint leftLo(0, rect.height, scaled, angle);
    cv::KeyPoint RightUp(rect.width, 0, scaled, angle);
    cv::KeyPoint RightLo(rect.width,rect.height, scaled, angle);

    if(soft_crop) {
        center.pt.x += rect.x;
        center.pt.y += rect.y;
        leftUp.pt.x += rect.x;
        leftUp.pt.y += rect.y;
        leftLo.pt.x += rect.x;
        leftLo.pt.y += rect.y;
        RightUp.pt.x += rect.x;
        RightUp.pt.y += rect.y;
        RightLo.pt.x += rect.x;
        RightLo.pt.y += rect.y;
    }

    key_points.push_back(center);
    key_points.push_back(leftUp);
    key_points.push_back(leftLo);
    key_points.push_back(RightUp);
    key_points.push_back(RightLo);

    return key_points;
}

CVExtractor::KeyPoints CVExtractor::prepareOctavedKeypoint(const Rect &rect, const float scale, const float angle)
{

}

PatternExtractor::PatternExtractor() :
    type(NOT_SET),
    k(0.0)
{
}

void PatternExtractor::extract(const cv::Mat &image, cv::Mat &descriptors)
{
    switch(type) {
    case LBP:
        extractLBP(image, descriptors);
        break;
    case LTP:
        extractLTP(image, descriptors);
        break;
    default:
        std::cerr << "No pattern set, computation not possible!" << std::endl;
    }
}

void PatternExtractor::set(cv_local_patterns::LBP *bp)
{
    type = LBP;
    pattern = bp;
}

void PatternExtractor::set(cv_local_patterns::LTP *tp)
{
    type = LTP;
    pattern = tp;
}

void PatternExtractor::setK(const double value)
{
    k = value;
}

void PatternExtractor::extractLBP(const Mat &image, Mat &descriptors)
{
    cv_local_patterns::LBP *ptr = static_cast<cv_local_patterns::LBP*>(pattern);
    ptr->stdExtraction<uchar>(image);
    descriptors = ptr->getHistogram();
}

void PatternExtractor::extractLTP(const Mat &image, Mat &descriptors)
{
    cv_local_patterns::LTP *ptr = static_cast<cv_local_patterns::LTP*>(pattern);
    ptr->stdExtraction<uchar>(image, k);
    cv::Mat pos = ptr->getPos();
    cv::Mat neg = ptr->getNeg();

    descriptors = cv::Mat_<int>(2, 256, 0);
    cv::Mat roi_pos(descriptors, cv::Rect(0,0,256,1));
    cv::Mat roi_neg(descriptors, cv::Rect(0,1,256,1));
    pos.copyTo(roi_pos);
    neg.copyTo(roi_neg);

}

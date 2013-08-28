#include "pattern_extractor.h"

using namespace cv_extraction;

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

void PatternExtractor::extract(const Mat &image,
                               const bool color_extension,
                               const bool large, Mat &descriptors)
{
    extract(image, descriptors);
    if(color_extension) {
        cv::Scalar  mean = extractMeanColorRGBHSV(image);

        if(large) {
            descriptors = descriptors.reshape(0, 1);
        }

        if(color_extension) {
            addColorExtension(descriptors, mean);
        }

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

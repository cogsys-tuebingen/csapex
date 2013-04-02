/// HEADER
#include "opencv_background_remover.h"

using namespace background_subtraction;

namespace
{
const int HISTORY = 16;
const float RATE = 0.002;
}

OpenCvBackgroundRemover::OpenCvBackgroundRemover()
    : subtractor(NULL)
{
    make();
}

void OpenCvBackgroundRemover::make()
{
    if(subtractor)
        delete subtractor;

    std::cout << "threshold: " << difference_threshold << std::endl;

    subtractor = new cv::BackgroundSubtractorMOG2(HISTORY, difference_threshold);
}

OpenCvBackgroundRemover::~OpenCvBackgroundRemover()
{
    if(subtractor)
        delete subtractor;
}

void OpenCvBackgroundRemover::segmentation(const cv::Mat& frame, cv::Mat& mask)
{
    if(changed){
        make();
        changed = false;
    }

    (*subtractor)(frame, mask, RATE);
    cv::threshold(mask, mask, 200, 255, CV_THRESH_BINARY);
}

cv::Mat OpenCvBackgroundRemover::getBackground()
{
    cv::Mat out;
    if(has_background){
        subtractor->getBackgroundImage(out);
    }
    return out;
}

void OpenCvBackgroundRemover::setBackground(const cv::Mat& frame)
{
    BackgroundRemover::setBackground(frame);

    make();
}

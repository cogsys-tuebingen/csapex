/// HEADER
#include "opencv_background_remover.h"

using namespace background_subtraction;

REGISTER_REMOVER(OpenCvBackgroundRemover);

OpenCvBackgroundRemover::OpenCvBackgroundRemover()
    : BackgroundRemover("OpenCV"), subtractor(NULL), has_frames(false), history(16), rate(0.002)
{
    make();
}

void OpenCvBackgroundRemover::make()
{
    if(subtractor)
        delete subtractor;

    std::cout << "threshold: " << difference_threshold << std::endl;

    subtractor = new cv::BackgroundSubtractorMOG2(history, difference_threshold);
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

    (*subtractor)(frame, mask, rate);
    cv::threshold(mask, mask, 200, 255, CV_THRESH_BINARY);

    has_frames = true;
}

cv::Mat OpenCvBackgroundRemover::getBackground()
{
    cv::Mat out;
    if(has_background && has_frames){
        subtractor->getBackgroundImage(out);
    } else {
        out = cv::Mat(10,10,CV_8U, cv::Scalar(255,127,0));
    }
    return out;
}

void OpenCvBackgroundRemover::setBackground(const cv::Mat& frame)
{
    BackgroundRemover::setBackground(frame);

    make();
}

void OpenCvBackgroundRemover::setHistory(int h)
{
    history = h;
    changed = true;
}

void OpenCvBackgroundRemover::setRate(float r)
{
    rate = r;
}

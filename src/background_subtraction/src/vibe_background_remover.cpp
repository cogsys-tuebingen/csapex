/// HEADER
#include "vibe_background_remover.h"

using namespace background_subtraction;

VibeBackgroundRemover::VibeBackgroundRemover()
{
}

VibeBackgroundRemover::~VibeBackgroundRemover()
{
    libvibeModelFree(model);
}

void VibeBackgroundRemover::init()
{
    BackgroundRemover::init();
    model = libvibeModelNew();
}

void VibeBackgroundRemover::setBackground(const cv::Mat& frame)
{
    BackgroundRemover::setBackground(frame);

    libvibeModelAllocInit_8u_C3R(model, background.data, background.cols, background.rows, background.step);
}

void VibeBackgroundRemover::segmentation(const cv::Mat& frame, cv::Mat& mask)
{
    cv::Mat blured;
    cv::GaussianBlur(frame, blured, blur_kernel, sigma_blur);
    libvibeModelUpdate_8u_C3R(model, blured.data, mask.data);
}

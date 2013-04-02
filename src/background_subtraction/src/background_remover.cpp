/// HEADER
#include "background_remover.h"

using namespace background_subtraction;

cv::Mat BackgroundRemover::NO_DEBUG = cv::Mat(5,5,CV_8UC3,cv::Scalar::all(0));

BackgroundRemover::BackgroundRemover()
    : has_background(false), difference_threshold(0),
      sigma_blur(5.5), blur_kernel(15,15),
      close_iterations(0), open_iterations(0),
      changed(false)
{
}

BackgroundRemover::~BackgroundRemover()
{

}

void BackgroundRemover::init()
{
}

void BackgroundRemover::setBackground(const cv::Mat& frame)
{
    background = frame;
    cv::GaussianBlur(frame, background_blured, blur_kernel, sigma_blur);

    has_background = true;
}

void BackgroundRemover::filter(const cv::Mat& frame, cv::Mat& filtered, cv::Mat& mask)
{
    // segmentation to produce a mask
    mask.create(frame.rows, frame.cols, CV_8U);
    segmentation(frame, mask);

    // morphing
    cv::Mat el = cv::getStructuringElement( 0, cv::Size(5, 5), cv::Point( 2, 2 ) );
    cv::morphologyEx(mask, mask, CV_MOP_CLOSE, el, cv::Point(-1, -1), close_iterations);
    cv::morphologyEx(mask, mask, CV_MOP_OPEN, el, cv::Point(-1, -1), open_iterations);

    // create output image
    createResult(filtered, frame, mask);

//    cv::imwrite("mask.png", mask);
//    cv::imwrite("filtered.png", filtered);
//    cv::imwrite("frame.png", frame);
}

void BackgroundRemover::createResult(cv::Mat& filtered, cv::Mat frame_blured, cv::Mat mask)
{
    filtered.create(frame_blured.rows, frame_blured.cols, frame_blured.type());

    uchar* mask_data = mask.data;
    uchar* filtered_data = filtered.data;
    uchar* frame_data = frame_blured.data;

    unsigned channels = filtered.channels();

    // actual filtering
    for (int i = 0; i < frame_blured.rows; i++) {
        for (int j = 0; j < frame_blured.cols; ++j) {
            unsigned char f = mask_data[i * mask.step + j];

            unsigned idx = i * filtered.step + j*channels;

            bool filter = f == 0;

            filtered_data[idx] = filter ? 0 : frame_data[idx];
            filtered_data[idx+1] = filter ? 0 : frame_data[idx+1];
            filtered_data[idx+2] = filter ? 0 : frame_data[idx+2];
        }
    }
}

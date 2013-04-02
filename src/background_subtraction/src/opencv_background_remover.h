#ifndef OPENCV_BACKGROUND_REMOVER_H
#define OPENCV_BACKGROUND_REMOVER_H

/// COMPONENT
#include "background_remover.h"

/// SYSTEM
#include <opencv2/video/background_segm.hpp>

namespace background_subtraction
{

/**
 * @brief The OpenCvBackgroundRemover class uses the OpenCV library for background subtraction
 */
class OpenCvBackgroundRemover : public BackgroundRemover
{
public:
    /**
     * @brief OpenCvBackgroundRemover
     */
    OpenCvBackgroundRemover();

    /**
     * @brief ~OpenCvBackgroundRemover
     */
    ~OpenCvBackgroundRemover();

    /**
     * @brief getBackground Accessor
     * @return background image
     */
    virtual cv::Mat getBackground();

    /**
     * @brief setBackground Setter
     * @param frame The frame to use as background
     */
    virtual void setBackground(const cv::Mat& frame);

protected:
    /**
     * @brief segmentation Template method for subclasses
     * @param original the current frame
     * @param mask Output: the generated mask
     */
    void segmentation(const cv::Mat& frame, cv::Mat& mask);

private:
    void make();

private:
    cv::BackgroundSubtractor* subtractor;
};
}

#endif // OPENCV_BACKGROUND_REMOVER_H

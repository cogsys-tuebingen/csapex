#ifndef VIBE_BACKGROUND_REMOVER_H
#define VIBE_BACKGROUND_REMOVER_H

/// COMPONENT
#include "background_remover.h"

extern "C" {
#include "vibe-background.h"
}

namespace background_subtraction
{

/**
 * @brief The VibeBackgroundRemover class adapts a third-party background removing class called vibe
 */
class VibeBackgroundRemover : public BackgroundRemover
{
public:
    /**
     * @brief VibeBackgroundRemover
     */
    VibeBackgroundRemover();

    /**
     * @brief ~VibeBackgroundRemover
     */
    ~VibeBackgroundRemover();

    /**
     * @brief init Initialization
     */
    void init();

    /**
     * @brief setBackground Override: capture background setting
     * @param frame the image to use as a background model
     */
    void setBackground(const cv::Mat& frame);

protected:
    /**
     * @brief segmentation Template method for subclasses
     * @param original the current frame
     * @param mask Output: the generated mask
     */
    void segmentation(const cv::Mat& frame, cv::Mat& mask);

private:
    void* stream;
    vibeModel_t* model;
};

}
#endif // VIBE_BACKGROUND_REMOVER_H

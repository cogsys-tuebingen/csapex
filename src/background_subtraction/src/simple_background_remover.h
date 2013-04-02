#ifndef SIMPLE_BACKGROUND_REMOVER_H
#define SIMPLE_BACKGROUND_REMOVER_H

/// COMPONENT
#include "background_remover.h"

namespace background_subtraction
{

/**
 * @brief The SimpleBackgroundRemover class implemens a naive background remover with a static background model
 */
class SimpleBackgroundRemover : public BackgroundRemover
{
public:
    /**
     * @brief SimpleBackgroundRemover
     */
    SimpleBackgroundRemover();

protected:
    /**
     * @brief segmentation Template method for subclasses
     * @param original the current frame
     * @param mask Output: the generated mask
     */
    void segmentation(const cv::Mat& frame, cv::Mat& mask);
};

}
#endif // SIMPLE_BACKGROUND_REMOVER_H

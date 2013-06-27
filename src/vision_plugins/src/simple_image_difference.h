#ifndef SIMPLE_IMAGE_DIFFERENCE_H
#define SIMPLE_IMAGE_DIFFERENCE_H

/// COMPONENT
#include <vision_evaluator/image_combiner.h>

namespace vision_plugins
{

class SimpleImageDifference : public vision_evaluator::ImageCombiner
{
public:
    SimpleImageDifference();

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);
    virtual void insert(QBoxLayout* layout);
};

}

#endif // SIMPLE_IMAGE_DIFFERENCE_H

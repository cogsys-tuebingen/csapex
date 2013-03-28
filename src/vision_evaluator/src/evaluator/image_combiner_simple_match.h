#ifndef IMAGE_COMBINER_SIMPLE_MATCH_H
#define IMAGE_COMBINER_SIMPLE_MATCH_H

/// COMPONENT
#include "image_combiner.h"

/// PROJECT
#include <config/reconfigurable.h>

class ImageCombinerSimpleMatch : public ImageCombiner, public Reconfigurable
{
    Q_OBJECT

public:
    ImageCombinerSimpleMatch(const std::string& label);

public:
    static ImageCombiner::TypePtr createInstance(CONSTRUCTOR_MODE mode);

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

};

#endif // IMAGE_COMBINER_SIMPLE_MATCH_H

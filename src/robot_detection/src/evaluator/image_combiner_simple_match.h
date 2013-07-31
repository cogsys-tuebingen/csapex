#ifndef IMAGE_COMBINER_SIMPLE_MATCH_H
#define IMAGE_COMBINER_SIMPLE_MATCH_H

/// COMPONENT
#include <csapex_vision/image_combiner.h>

/// PROJECT
#include <config/reconfigurable.h>

namespace robot_detection
{

class ImageCombinerSimpleMatch : public csapex::ImageCombiner, public Reconfigurable
{
    Q_OBJECT

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

};

}

#endif // IMAGE_COMBINER_SIMPLE_MATCH_H

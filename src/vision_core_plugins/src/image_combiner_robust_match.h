/*
 * image_combiner_robust_match.h
 *
 *  Created on: Feb 7, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// PROJECT
#include <vision_evaluator/image_combiner.h>

#ifndef IMAGE_COMBINER_ROBUST_MATCH_H
#define IMAGE_COMBINER_ROBUST_MATCH_H

namespace csapex
{

class ImageCombinerRobustMatch : public ImageCombiner
{
    Q_OBJECT

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_ROBUST_MATCH_H

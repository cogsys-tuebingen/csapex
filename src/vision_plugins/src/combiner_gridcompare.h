#ifndef COMBINER_GRIDCOMPARE_H
#define COMBINER_GRIDCOMPARE_H

#include <vision_evaluator/image_combiner.h>

namespace vision_evaluator {
class GridCompare : public ImageCombiner
{
public:
    GridCompare();
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

protected:
    virtual void fill(QBoxLayout *layout);
};
}
#endif // COMBINER_GRIDCOMPARE_H

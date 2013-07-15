#ifndef COMBINER_GRIDCOMPARE_VALUE_H
#define COMBINER_GRIDCOMPARE_VALUE_H

/// COMPONENT
#include "combiner_gridcompare.h"

namespace vision_evaluator {
/**
 * @brief The GridCompareValue class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the mean value of values given
 *        in a grid cell.
 */
class GridCompareValue : public GridCompare
{
public:
    GridCompareValue();

    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

protected:
    virtual void fill(QBoxLayout *layout);
};
}
#endif // COMBINER_GRIDCOMPARE_VALUE_H

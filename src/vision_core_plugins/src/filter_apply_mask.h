#ifndef FILTER_APPLY_MASK_H
#define FILTER_APPLY_MASK_H

/// PROJECT
#include <vision_evaluator/filter.h>

namespace csapex
{
class FilterApplyMask : public Filter
{
    Q_OBJECT

public:
    FilterApplyMask();

    virtual void filter(cv::Mat& img, cv::Mat& mask);

    virtual void insert(QBoxLayout* layout);
};
}

#endif // FILTER_APPLY_MASK_H

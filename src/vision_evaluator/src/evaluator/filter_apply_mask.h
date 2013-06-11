#ifndef FILTER_APPLY_MASK_H
#define FILTER_APPLY_MASK_H

/// COMPONENT
#include "filter.h"

namespace vision_evaluator
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

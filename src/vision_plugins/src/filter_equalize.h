#ifndef FILTER_EUQALIZE_H
#define FILTER_EQUALIZE_H

/// COMPONENT
#include <vision_evaluator/filter.h>

namespace vision_plugins {
/**
 * @brief The equalization plugin represents the cv function to euqalize an image using histograms.
 */
class Equalize : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    Equalize();

    virtual ~Equalize();

    virtual void insert(QBoxLayout *parent);
    virtual void filter(cv::Mat &img, cv::Mat &mask);
protected:
    virtual bool usesMask();

};
}



#endif // FILTER_EQUALIZE_H

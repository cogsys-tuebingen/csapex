#ifndef FilterBlur_H
#define FilterBlur_H

/// COMPONENT
#include "filter.h"

/// SYSTEM
#include <QSlider>

namespace vision_evaluator
{

class FilterBlur : public Filter
{
    Q_OBJECT

public:
    FilterBlur();

    virtual void filter(cv::Mat& img, cv::Mat& mask);

    virtual void insert(QBoxLayout* layout);


private Q_SLOTS:
    void update(int slot);

private:
    QSlider* slider;
    int blur;

};

} /// NAMESPACE

#endif // FilterBlur_H

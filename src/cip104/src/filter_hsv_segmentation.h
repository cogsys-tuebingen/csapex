#ifndef FILTER_TOOL_DETECTION_H
#define FILTER_TOOL_DETECTION_H

/// COMPONENT
#include <vision_evaluator/filter.h>

/// SYSTEM
#include <QSlider>

namespace cip104
{

class HSVSegmentation : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    HSVSegmentation();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask);
    virtual void insert(QBoxLayout* layout);

public Q_SLOTS:
    void update();

private:
    QSlider* min_h;
    QSlider* min_s;
    QSlider* min_v;
    QSlider* max_h;
    QSlider* max_s;
    QSlider* max_v;

    cv::Scalar min;
    cv::Scalar max;
};

}

#endif // FILTER_TOOL_DETECTION_H

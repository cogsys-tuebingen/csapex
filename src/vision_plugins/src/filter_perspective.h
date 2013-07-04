#ifndef FILTER_PERSPECTIVE_H
#define FILTER_PERSPECTIVE_H

/// COMPONENT
#include <vision_evaluator/filter.h>
#include <utils/LibCvTools/perspective_transform.h>

/// SYSTEM
class QDoubleSlider;

namespace vision_plugins {

class PerspectiveTransform : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    PerspectiveTransform();

    virtual void filter(cv::Mat &img, cv::Mat &mask);
    virtual void insert(QBoxLayout *parent);

public Q_SLOTS:
    void update();


private:
    PerspectiveTransformer transformer_;

    QDoubleSlider *rot_x_;
    QDoubleSlider *rot_y_;
    QDoubleSlider *rot_z_;
    QDoubleSlider *distance_;
    QDoubleSlider *focal_length_;

};

}
#endif // FILTER_PERSPECTIVE_H

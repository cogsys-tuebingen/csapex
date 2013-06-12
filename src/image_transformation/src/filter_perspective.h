#ifndef FILTER_PERSPECTIVE_H
#define FILTER_PERSPECTIVE_H

/// COMPONENT
#include <vision_evaluator/filter.h>
#include "perspective.h"

/// SYSTEM
#include <QSlider>

namespace image_transformation {

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

    QSlider *rot_x_;
    QSlider *rot_y_;
    QSlider *rot_z_;
    QSlider *distance_;
    QSlider *focal_length_;

};

}
#endif // FILTER_PERSPECTIVE_H

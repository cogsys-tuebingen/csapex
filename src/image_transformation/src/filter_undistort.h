#ifndef FILTER_UNDISTORT_H
#define FILTER_UNDISTORT_H

/// COMPONENT
#include <vision_evaluator/filter.h>
#include "undistort.h"

/// SYSTEM
#include <QSlider>
#include <QFileDialog>

namespace image_transformation {
class Undistort : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    Undistort();

    virtual void filter(cv::Mat &img, cv::Mat &mask);
    virtual void insert(QBoxLayout *parent);

public Q_SLOTS:
    void update();

private:
    Undistorter *undist;
};

}

#endif // FILTER_UNDISTORT_H

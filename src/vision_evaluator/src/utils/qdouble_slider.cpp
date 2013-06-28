#include "qdouble_slider.h"
#include <iostream>

QDoubleSlider::QDoubleSlider(Qt::Orientation orientation, double step_size, QWidget *parent) :
    QSlider(orientation, parent),
    step_(step_size),
    min_(0.0),
    max_(1.0)
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(scaleValue(int)));
}

QDoubleSlider::~QDoubleSlider()
{
}

void QDoubleSlider::update()
{
    setMaximum(double2int(max_));
}

double QDoubleSlider::int2double(int val)
{
    return val * step_ + min_;
}

int QDoubleSlider::double2int(double val)
{
    return (val - min_) / step_;
}


void QDoubleSlider::scaleValue(int value)
{
    Q_EMIT valueChanged(int2double(value));
}

void QDoubleSlider::setDoubleMinimum(double min)
{
    min_ = min;
    update();
}

void QDoubleSlider::setDoubleMaximum(double max)
{
    max_ = max;
    update();
}

void QDoubleSlider::setDoubleValue(double val)
{
    setValue(double2int(val));
}

double QDoubleSlider::doubleValue()
{
    return int2double(value());
}

double QDoubleSlider::doubleMaximum()
{
    return int2double(maximum());
}

double QDoubleSlider::doubleMinimum()
{
    return int2double(minimum());
}

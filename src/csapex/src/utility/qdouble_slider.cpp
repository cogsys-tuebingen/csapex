/// HEADER
#include <csapex/utility/qdouble_slider.h>

/// SYSTEM
#include <iostream>
#include <limits>
#include <cmath>

QDoubleSlider::QDoubleSlider(Qt::Orientation orientation, double step_size, QWidget *parent) :
    QSlider(orientation, parent),
    step_(step_size),
    min_(0.0),
    max_(1.0)
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(scaleValue(int)));
    connect(this, SIGNAL(rangeChanged(int,int)), this, SLOT(emitRangeChanged(int,int)));
}

QDoubleSlider::~QDoubleSlider()
{
}

void QDoubleSlider::update()
{
    if(min_ > max_) {
        max_ = min_;
    }
    setRange(double2int(min_), double2int(max_));
}

double QDoubleSlider::int2double(int val)
{
    return val * step_ + min_;
}

int QDoubleSlider::double2int(double val)
{
    return round((val - min_) / step_);
}


void QDoubleSlider::scaleValue(int value)
{
    double val = int2double(value);
    if(val != doubleValue()){
        setDoubleValue(val);
    }

    Q_EMIT doubleValueChanged(val);
}

void QDoubleSlider::setDoubleMinimum(double min)
{
    bool change = min != min_;

    if(change) {
        min_ = min;
        update();
    }
}

void QDoubleSlider::setDoubleMaximum(double max)
{
    bool change = max != max_;

    if(change) {
        max_ = max;
        update();
    }
}

void QDoubleSlider::setDoubleRange(double min, double max)
{
    bool change = max != max_ || min != min_;

    if(change) {
        max_ = max;
        min_ = min;
        update();
    }
}

void QDoubleSlider::setDoubleValue(double val)
{
    int intval= double2int(val);
    if(value() != intval) {
        setValue(intval);
    }
}

void QDoubleSlider::setNearestDoubleValue(double val)
{
    int intval= double2int(val);
    if(value() != intval) {
        blockSignals(true);
        setValue(intval);
        blockSignals(false);

        Q_EMIT doubleValueChanged(val);
    }
}

void QDoubleSlider::limitMin(double limit)
{
    double limited = std::max(doubleValue(), limit);
    setDoubleValue(limited);
}

void QDoubleSlider::limitMax(double limit)
{
    double limited = std::min(doubleValue(), limit);
    setDoubleValue(limited);
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

void QDoubleSlider::emitRangeChanged(int min, int max)
{
    if(int2double(min) != min_)
        setDoubleMinimum(int2double(min));
    if(int2double(max) != max_)
        setDoubleMaximum(int2double(max));

    Q_EMIT doubleRangeChanged(min_, max_);
}

/// HEADER
#include <csapex/utility/qdouble_slider.h>

/// SYSTEM
#include <iostream>
#include <limits>

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
    double val = int2double(value);
    if(val != doubleValue()){
        setDoubleValue(val);
    }

    Q_EMIT valueChanged(val);
}

void QDoubleSlider::setDoubleMinimum(double min)
{
    bool change = min != min_;
    min_ = min;

    if(change) {
        update();
    }
}

void QDoubleSlider::setDoubleMaximum(double max)
{
    bool change = max != max_;
    max_ = max;

    if(change) {
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

        Q_EMIT valueChanged(val);
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

    Q_EMIT rangeChanged(min_, max_);
}

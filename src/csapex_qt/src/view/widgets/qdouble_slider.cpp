/// HEADER
#include <csapex/view/widgets/qdouble_slider.h>

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

    update();
}

QDoubleSlider::~QDoubleSlider()
{
}

void QDoubleSlider::setStepSize(double step)
{
    step_ = step;
    update();
}

void QDoubleSlider::update()
{
    if(min_ > max_) {
        max_ = min_;
    }
    setSingleStep(step_);
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
    if(val != scaledValue()){
        setScaledValue(val);
    }

    Q_EMIT scaledValueChanged(val);
}

void QDoubleSlider::setScaledMinimum(double min)
{
    bool change = min != min_;

    if(change) {
        min_ = min;
        update();
    }
}

void QDoubleSlider::setScaledMaximum(double max)
{
    bool change = max != max_;

    if(change) {
        max_ = max;
        update();
    }
}

void QDoubleSlider::setScaledRange(double min, double max)
{
    bool change = max != max_ || min != min_;

    if(change) {
        max_ = max;
        min_ = min;
        update();
    }
}

void QDoubleSlider::setScaledValue(double val)
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

        Q_EMIT scaledValueChanged(val);
    }
}

void QDoubleSlider::limitMin(double limit)
{
    double limited = std::max(scaledValue(), limit);
    setScaledValue(limited);
}

void QDoubleSlider::limitMax(double limit)
{
    double limited = std::min(scaledValue(), limit);
    setScaledValue(limited);
}


double QDoubleSlider::scaledValue()
{
    return int2double(value());
}

double QDoubleSlider::scaledMaximum()
{
    return int2double(maximum());
}

double QDoubleSlider::scaledMinimum()
{
    return int2double(minimum());
}

void QDoubleSlider::emitRangeChanged(int min, int max)
{
    if(int2double(min) != min_)
        setScaledMinimum(int2double(min));
    if(int2double(max) != max_)
        setScaledMaximum(int2double(max));

    Q_EMIT scaledRangeChanged(min_, max_);
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_qdouble_slider.cpp"

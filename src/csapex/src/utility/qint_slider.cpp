/// HEADER
#include <csapex/utility/qint_slider.h>

/// SYSTEM
#include <iostream>
#include <limits>

QIntSlider::QIntSlider(Qt::Orientation orientation, double step_size, QWidget *parent) :
    QSlider(orientation, parent),
    step_(step_size),
    min_(0),
    max_(1)
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(scaleValue(int)));
    connect(this, SIGNAL(rangeChanged(int,int)), this, SLOT(emitRangeChanged(int,int)));
}

QIntSlider::~QIntSlider()
{
}

void QIntSlider::update()
{
    if(min_ > max_) {
        max_ = min_;
    }
    int min = integer2int(min_);
    int max = integer2int(max_);

    if(max != maximum() || min != minimum()) {
        setRange(min, max);
    }
}

int QIntSlider::int2integer(int val)
{
    return val * step_ + min_;
}

int QIntSlider::integer2int(int val)
{
    return (val - min_) / step_;
}


void QIntSlider::scaleValue(int value)
{
    double val = int2integer(value);
    if(val != intValue()){
        setIntValue(val);
    }

    Q_EMIT intValueChanged(val);
}

void QIntSlider::setIntMinimum(int min)
{
    bool change = min != min_;
    min_ = min;

    if(min_ > max_) {
        max_ = min_;
    }

    if(change) {
        update();
    }
}

void QIntSlider::setIntMaximum(int max)
{
    bool change = max != max_;
    max_ = max;

    if(change) {
        update();
    }
}

void QIntSlider::setIntValue(int val)
{
    int intval= integer2int(val);
    if(value() != intval) {
        setValue(intval);
    }
}

void QIntSlider::limitMin(int limit)
{
    double limited = std::max(intValue(), limit);
    setIntValue(limited);
}

void QIntSlider::limitMax(int limit)
{
    double limited = std::min(intValue(), limit);
    setIntValue(limited);
}


int QIntSlider::intValue()
{
    return int2integer(value());
}

int QIntSlider::intMaximum()
{
    return int2integer(maximum());
}

int QIntSlider::intMinimum()
{
    return int2integer(minimum());
}

void QIntSlider::emitRangeChanged(int min, int max)
{
    if(int2integer(min) != min_)
        setIntMinimum(int2integer(min));
    if(int2integer(max) != max_)
        setIntMaximum(int2integer(max));

    Q_EMIT intRangeChanged(min_, max_);
}

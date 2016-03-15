/// HEADER
#include <csapex/view/widgets/doublespanslider.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>
#include <cmath>

DoubleSpanSlider::DoubleSpanSlider(Qt::Orientation orientation, double step_size, QWidget* parent)
    : QxtSpanSlider(orientation, parent),
      step_(step_size),
      min_(0.0),
      max_(1.0)
{
    connect(this, SIGNAL(spanChanged(int,int)), this, SLOT(scaleSpan(int,int)));
    connect(this, SIGNAL(rangeChanged(int,int)), this, SLOT(scaleRange(int,int)));
}

void DoubleSpanSlider::setSpan(double lower, double upper)
{
    QxtSpanSlider::setSpan(double2int(lower), double2int(upper));
}

double DoubleSpanSlider::lowerDoubleValue() const
{
    return int2double(lowerValue());
}
double DoubleSpanSlider::upperDoubleValue() const
{
    return int2double(upperValue());
}
double DoubleSpanSlider::doubleMinimum() const
{
    return min_;
}
double DoubleSpanSlider::doubleMaximum() const
{
    return max_;
}

void DoubleSpanSlider::setLowerDoubleValue(double lower)
{
    setLowerValue(double2int(lower));
}
void DoubleSpanSlider::setUpperDoubleValue(double upper)
{
    setUpperValue(double2int(upper));
}

void DoubleSpanSlider::setDoubleMinimum(double min)
{
    update(min, max_);
}
void DoubleSpanSlider::setDoubleMaximum(double max)
{
    update(min_, max);
}

void DoubleSpanSlider::setDoubleRange(double min, double max)
{
    update(min, max);
}

void DoubleSpanSlider::setRange(double min, double max)
{
    update(min, max);
}

void DoubleSpanSlider::update(double min, double max)
{
    apex_assert_hard(step_ != 0.0);

    int mini = double2int(min, min);
    int maxi = double2int(max, min);

    if(minimum() != mini || maximum() != maxi) {
        double low = int2double(lowerValue());
        double up = int2double(upperValue());

        apex_assert_hard(int2double(maxi, min) == max);
        apex_assert_hard(int2double(mini, min) == min);

        min_ = min;
        max_ = max;

        QxtSpanSlider::setRange(mini, maxi);

        setLowerValue(double2int(low));
        setUpperValue(double2int(up));
   }
}


void DoubleSpanSlider::scaleSpan(int l, int u)
{
    double low = int2double(l);
    double up = int2double(u);

    if(low != lowerDoubleValue()){
        setLowerDoubleValue(low);
    }
    if(up != upperDoubleValue()){
        setUpperDoubleValue(up);
    }

    Q_EMIT lowerValueChanged(low);
    Q_EMIT upperValueChanged(up);
    Q_EMIT spanChanged(low, up);
}

void DoubleSpanSlider::scaleRange(int l, int u)
{
    double low = int2double(l);
    double up = int2double(u);

    update(low, up);

    Q_EMIT rangeChanged(low, up);
}

double DoubleSpanSlider::int2double(int val) const
{
    return int2double(val, min_);
}

int DoubleSpanSlider::double2int(double val) const
{
    return double2int(val, min_);
}


double DoubleSpanSlider::int2double(int val, double min) const
{
    return val * step_ + min;
}

int DoubleSpanSlider::double2int(double val, double min) const
{
    return (val - min) / step_;
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_doublespanslider.cpp"

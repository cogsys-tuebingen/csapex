/// HEADER
#include <csapex/view/utility/qsignal_bridges.h>

using namespace QSignalBridges;

QAbstractSliderLimiter::QAbstractSliderLimiter(QAbstractSlider *limitMin, QAbstractSlider *limitMax) :
    limitMin_(limitMin),
    limitMax_(limitMax)
{
    QObject::connect(limitMin_, SIGNAL(valueChanged(int)), this, SLOT(limitMin(int)));
    QObject::connect(limitMax_, SIGNAL(valueChanged(int)), this, SLOT(limitMax(int)));
}

QAbstractSliderLimiter::~QAbstractSliderLimiter()
{
}

void QAbstractSliderLimiter::limitMin(int limit)
{
    int limited = std::max(limitMax_->value(), limit);
    limitMax_->setValue(limited);
}

void QAbstractSliderLimiter::limitMax(int limit)
{
    int limited = std::min(limitMin_->value(), limit);
    limitMin_->setValue(limited);
}
/// MOC
#include "../../../include/csapex/view/utility/moc_qsignal_bridges.cpp"

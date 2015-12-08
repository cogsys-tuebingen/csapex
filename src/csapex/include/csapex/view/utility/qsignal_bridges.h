#ifndef QSIGNAL_BRIDGES_H
#define QSIGNAL_BRIDGES_H

/// SYSTEM
#include <QObject>
#include <QAbstractSlider>
#include <memory>

namespace QSignalBridges
{
class QAbstractSliderLimiter : public QObject {

    Q_OBJECT

public:
    QAbstractSliderLimiter(QAbstractSlider* limitMin, QAbstractSlider* limitMax);
    virtual ~QAbstractSliderLimiter();

    typedef std::shared_ptr<QAbstractSliderLimiter> Ptr;

private Q_SLOTS:
    void limitMin(int limit);
    void limitMax(int limit);

private:
    QAbstractSlider *limitMin_;
    QAbstractSlider *limitMax_;
};
}
#endif // QSIGNAL_BRIDGES_H




#ifndef QSIGNAL_BRIDGES_H
#define QSIGNAL_BRIDGES_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QObject>
#include <QAbstractSlider>
#include <memory>

namespace QSignalBridges
{
class CSAPEX_QT_EXPORT QAbstractSliderLimiter : public QObject {

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




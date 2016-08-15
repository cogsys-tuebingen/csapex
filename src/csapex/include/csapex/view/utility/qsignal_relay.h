#ifndef qsignal_relay_H
#define qsignal_relay_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QObject>
#include <functional>

namespace qt_helper {

class CSAPEX_QT_EXPORT Call : public QObject
{
    Q_OBJECT

    typedef std::function<void()> CB;

public:
    Call(CB cb)
        : cb_(cb), valid_(true)
    {}

public Q_SLOTS:
    void call() {
        if(valid_) {
            cb_();
        }
    }

    void invalidate() {
        valid_ = false;
    }

private:
    CB cb_;
    bool valid_;
};

}
#endif // qsignal_relay_H

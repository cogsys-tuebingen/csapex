#ifndef qsignal_relay_H
#define qsignal_relay_H

/// SYSTEM
#include <QObject>
#include <functional>

namespace qt_helper {

class Call : public QObject
{
    Q_OBJECT

    typedef std::function<void()> CB;

public:
    Call(CB cb)
        : cb_(cb)
    {}

public Q_SLOTS:
    void call() {
        cb_();
    }

private:
    CB cb_;
};

}
#endif // qsignal_relay_H

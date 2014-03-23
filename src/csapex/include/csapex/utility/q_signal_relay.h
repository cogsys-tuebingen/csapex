#ifndef Q_SIGNAL_RELAY_H
#define Q_SIGNAL_RELAY_H

/// SYSTEM
#include <QObject>
#include <boost/function.hpp>

namespace qt_helper {

struct Call : public QObject
{
    Q_OBJECT

    typedef boost::function<void()> CB;

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
#endif // Q_SIGNAL_RELAY_H

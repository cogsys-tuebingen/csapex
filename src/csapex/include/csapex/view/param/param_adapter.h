#ifndef PARAM_ADAPTER_H
#define PARAM_ADAPTER_H

/// PROJECT
#include <csapex/view/utility/qsignal_relay.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/param/parameter.h>
#include <csapex/param/proxy.hpp>

/// SYSTEM
#include <string>
#include <QObject>

class QBoxLayout;

namespace csapex
{

class ParameterAdapter : public QObject
{
    Q_OBJECT

public:
    ParameterAdapter(param::Proxy<param::Parameter>::Ptr p);
    virtual ~ParameterAdapter();

    virtual void setup(QBoxLayout* layout, const std::string& display_name) = 0;

Q_SIGNALS:
    void modelCallback(std::function<void()>);

public Q_SLOTS:
    void executeModelCallback(std::function<void()>);

protected:
    void connectInGuiThread(csapex::slim_signal::Signal<void(csapex::param::Parameter*)>& signal,
                 std::function<void()> cb);

protected:
    param::Proxy<param::Parameter>::Ptr p_;

private:
    std::vector<csapex::slim_signal::ScopedConnection> connections;

};


}

#endif // PARAM_ADAPTER_H

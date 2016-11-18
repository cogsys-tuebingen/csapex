#ifndef PARAM_ADAPTER_H
#define PARAM_ADAPTER_H

/// PROJECT
#include <csapex/utility/slim_signal.h>
#include <csapex/param/parameter.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <string>
#include <QObject>

class QBoxLayout;
class QHBoxLayout;


namespace csapex
{

class ParameterContextMenu;

class CSAPEX_QT_EXPORT ParameterAdapter : public QObject
{
    Q_OBJECT

public:
    ParameterAdapter(param::Parameter::Ptr p);
    virtual ~ParameterAdapter();

    void doSetup(QBoxLayout* layout, const std::string& display_name);

protected:
    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) = 0;
    virtual void setupContextMenu(ParameterContextMenu* context_handler);

    void disconnect();

public:
    csapex::slim_signal::Signal<void(CommandPtr)> executeCommand;

Q_SIGNALS:
    void modelCallback(std::function<void()>);
    void customContextMenuRequested(QWidget*, QPoint);

public Q_SLOTS:
    void executeModelCallback(std::function<void()>);

protected:
    template <typename Callback, typename T>
    void connectInGuiThread(csapex::slim_signal::Signal<void(T)>& signal,
                            Callback cb)
    {
        connections.push_back(signal.connect([=](T v) {
            modelCallback([=]() {
                cb(v);
            });
        }));
    }

protected:
    param::Parameter::Ptr p_;

    ParameterContextMenu* context_handler;

private:
    std::vector<csapex::slim_signal::ScopedConnection> connections;

};


}

#endif // PARAM_ADAPTER_H

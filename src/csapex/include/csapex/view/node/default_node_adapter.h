#ifndef DEFAULT_NODE_ADAPTER_H
#define DEFAULT_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/view/node/node_adapter.h>

/// PROJECT
#include <csapex/param/param_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/memento.h>

/// SYSTEM
#include <QObject>

/// FORWARD DECLARTIONS
class QComboBox;
class QGroupBox;

namespace qt_helper{
class Call;
}

namespace csapex
{

typedef std::function<void()> Function;

class DefaultNodeAdapter;

class CSAPEX_QT_EXPORT DefaultNodeAdapterBridge : public QObject
{
    Q_OBJECT

    friend class DefaultNodeAdapter;

public:
    DefaultNodeAdapterBridge(DefaultNodeAdapter* parent);
    ~DefaultNodeAdapterBridge();

    void connectInGuiThread(csapex::slim_signal::Signal<void(csapex::param::Parameter*)>& signal,
                 std::function<void()> cb);
    void disconnect();

public Q_SLOTS:
    void setupAdaptiveUi();

    void enableGroup(bool enable, const std::string& group);

    void executeModelCallback(Function);

Q_SIGNALS:
    void setupAdaptiveUiRequest();

    void modelCallback(Function);

public:
    void triggerSetupAdaptiveUiRequest();

private:
    DefaultNodeAdapter* parent_;
    std::vector<csapex::slim_signal::Connection> connections;
};

class CSAPEX_QT_EXPORT DefaultNodeAdapter : public NodeAdapter
{
    friend class DefaultNodeAdapterBridge;

public:
    DefaultNodeAdapter(NodeHandleWeakPtr adaptee, NodeBox* parent);
    virtual ~DefaultNodeAdapter();

    virtual void stop();

public:
    void setupParameter(param::TriggerParameterPtr trigger_p);

    template <typename Parameter, typename Adapter>
    void setupParameter(std::shared_ptr<Parameter> adapter);


protected:
    virtual void setupAdaptiveUi();
    virtual void setupUi(QBoxLayout* layout);

    void clear();

public:
    DefaultNodeAdapterBridge bridge;

private:
    qt_helper::Call* makeModelCall(std::function<void()> cb);

private:
    std::vector<csapex::slim_signal::ScopedConnection> connections_;

    std::vector<QObject*> callbacks;
    std::map<std::string, QBoxLayout*> groups;
    std::map<std::string, bool> groups_enabled;
    std::map<std::string, QGroupBox*> groupsboxes;

    std::vector<ParameterAdapterPtr> adapters_;

    std::map<Connectable*, csapex::slim_signal::ScopedConnection> parameter_connections_;

    QBoxLayout* wrapper_layout_;
};

}

#endif // DEFAULT_NODE_ADAPTER_H

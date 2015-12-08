#ifndef DEFAULT_NODE_ADAPTER_H
#define DEFAULT_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/view/node_adapter.h>

/// PROJECT
#include <utils_param/param_fwd.h>

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

class DefaultNodeAdapterBridge : public QObject
{
    Q_OBJECT

public:
    DefaultNodeAdapterBridge(DefaultNodeAdapter* parent);

    void connectInGuiThread(boost::signals2::signal<void(param::Parameter*)>& signal,
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
    std::vector<boost::signals2::connection> connections;
};

class DefaultNodeAdapter : public NodeAdapter
{
    friend class DefaultNodeAdapterBridge;

public:
    DefaultNodeAdapter(NodeWorker* adaptee, WidgetController* widget_ctrl);
    virtual ~DefaultNodeAdapter();

    virtual void stop();

public:
    // TODO: WeakPtrs!!!!
    void setupParameter(param::TriggerParameter* trigger_p);
    void setupParameter(param::ColorParameter* color_p);
    void setupParameter(param::PathParameter* path_p);
    void setupParameter(param::ValueParameter* value_p);
    void setupParameter(param::RangeParameter* range_p);
    void setupParameter(param::IntervalParameter* interval_p);
    void setupParameter(param::SetParameter* set_p);
    void setupParameter(param::BitSetParameter* bitset_p);

    void setupParameter(param::OutputProgressParameter* bitset_p);


protected:
    virtual void setupAdaptiveUi();
    virtual void setupUi(QBoxLayout* layout);

    void clear();

public:
    DefaultNodeAdapterBridge bridge;

private:
    qt_helper::Call* makeModelCall(std::function<void()> cb);
    qt_helper::Call* makeUiCall(std::function<void()> cb);
    qt_helper::Call* makePausedUiCall(std::function<void()> cb);

private:
    std::vector<QObject*> callbacks;
    std::map<std::string, QBoxLayout*> groups;
    std::map<std::string, bool> groups_enabled;
    std::map<std::string, QGroupBox*> groupsboxes;

    std::map<Connectable*, boost::signals2::connection> parameter_connections_;

    QBoxLayout* wrapper_layout_;
};

}

#endif // DEFAULT_NODE_ADAPTER_H

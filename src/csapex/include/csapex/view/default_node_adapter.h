#ifndef DEFAULT_NODE_ADAPTER_H
#define DEFAULT_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/view/node_adapter.h>
#include <csapex/utility/context_menu_handler.h>

/// PROJECT
#include <utils_param/param_fwd.h>

/// FORWARD DECLARTIONS
class QComboBox;

namespace csapex
{

class DefaultNodeAdapter;

class DefaultNodeAdapterBridge : public QObject
{
    Q_OBJECT

public:
    DefaultNodeAdapterBridge(DefaultNodeAdapter* parent);

public Q_SLOTS:
    void modelChangedEvent();
    void setupAdaptiveUi();

    void enableGroup(bool enable, const std::string& group);

Q_SIGNALS:
    void guiChanged();
    void setupAdaptiveUiRequest();

public:
    void triggerGuiChanged();
    void triggerSetupAdaptiveUiRequest();

private:
    DefaultNodeAdapter* parent_;
};


class DefaultNodeAdapter : public NodeAdapter
{
    friend class DefaultNodeAdapterBridge;

public:
    DefaultNodeAdapter(Node* adaptee, WidgetController* widget_ctrl);
    virtual ~DefaultNodeAdapter();

    virtual void stop();
    virtual void guiChanged();

protected:
    virtual void setupAdaptiveUi();
    virtual void setupUi(QBoxLayout* layout);

    template <typename T>
    void updateParam(const std::string& name, T value);

    void updateParamIfNotEmpty(const std::string& name, const std::string& value);

    void updateParamSet(const std::string& name, const std::string& value);
    void updateParamBitSet(const std::string& name, const QListView *list);

    template <typename T>
    void updateUi(const param::Parameter* p, boost::function<void(T)> setter);
    template <typename T>
    void updateUiPtr(const param::Parameter* p, boost::function<void(const T*)> setter);

    void updateUiSet(const param::Parameter* p, boost::function<void(const std::string&)> setter);
    void updateUiBitSet(const param::Parameter* p, const QListView *list);

    void updateUiSetScope(const param::SetParameter* set_p, QComboBox* combo);

protected:
    void setupParameter(param::TriggerParameter* trigger_p);
    void setupParameter(param::ColorParameter* color_p);
    void setupParameter(param::PathParameter* path_p);
    void setupParameter(param::ValueParameter* value_p);
    void setupParameter(param::RangeParameter* range_p);
    void setupParameter(param::IntervalParameter* interval_p);
    void setupParameter(param::SetParameter* set_p);
    void setupParameter(param::BitSetParameter* bitset_p);

    void clear();


public:
    DefaultNodeAdapterBridge bridge;

private:
    std::vector<QObject*> callbacks;
    std::map<std::string, QBoxLayout*> groups;
    std::map<std::string, bool> groups_enabled;
    std::vector<boost::signals2::connection> connections;

    QBoxLayout* wrapper_layout_;
};

}

#endif // DEFAULT_NODE_ADAPTER_H

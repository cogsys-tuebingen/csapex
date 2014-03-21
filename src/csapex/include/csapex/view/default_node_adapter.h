#ifndef DEFAULT_NODE_ADAPTER_H
#define DEFAULT_NODE_ADAPTER_H

/// SYSTEM
#include <csapex/view/node_adapter.h>

namespace csapex
{

class DefaultNodeAdapter : public NodeAdapter
{
public:
    DefaultNodeAdapter(Node* adaptee);
    virtual ~DefaultNodeAdapter();

protected:
    virtual void setupUi(QBoxLayout* layout);

    template <typename T>
    void updateParam(const std::string& name, T value);

    void updateParamSet(const std::string& name, const std::string& value);
    void updateParamBitSet(const std::string& name, const QListView *list);

    template <typename T>
    void updateUi(const param::Parameter* p, boost::function<void(T)> setter);

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

private:
    std::vector<QObject*> callbacks;
    std::vector<boost::signals2::connection> connections;
};

}

#endif // DEFAULT_NODE_ADAPTER_H

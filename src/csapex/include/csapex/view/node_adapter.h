#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QLayout>
#include <memory>
#include <boost/signals2.hpp>
#include <vector>

class QListView;

namespace csapex
{
class NodeAdapter
{
public:
    typedef std::shared_ptr<NodeAdapter> Ptr;

protected:
    NodeAdapter(NodeWorker* adaptee, WidgetController *widget_ctrl);

public:
    virtual ~NodeAdapter();

    NodeWorker *getNodeWorker();

    void doSetupUi(QBoxLayout* layout);

    virtual void nodeModelChangedEvent();
    virtual void updateDynamicGui(QBoxLayout* layout);

    virtual void stop();

    virtual MementoPtr getState() const;
    virtual void setParameterState(MementoPtr memento);

protected:
    virtual void setupUi(QBoxLayout* layout) = 0;

protected:
    QBoxLayout *layout_;
    bool is_gui_setup_;

    std::string current_name_;
    std::string current_display_name_;
    QBoxLayout* current_layout_;

    NodeWorker* node_;
    WidgetController* widget_ctrl_;
};

}

#endif // NODE_ADAPTER_H

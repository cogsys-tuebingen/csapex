#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QLayout>
#include <memory>
#include <csapex/utility/slim_signal.hpp>
#include <vector>

class QListView;

namespace csapex
{
class NodeAdapter
{
public:
    typedef std::shared_ptr<NodeAdapter> Ptr;

protected:
    NodeAdapter(NodeHandleWeakPtr adaptee, WidgetController *widget_ctrl);

public:
    virtual ~NodeAdapter();

    void doSetupUi(QBoxLayout* layout);

    virtual void stop();

    virtual MementoPtr getState() const;
    virtual void setParameterState(MementoPtr memento);

protected:
    virtual void setupUi(QBoxLayout* layout) = 0;
    void trackConnection(const csapex::slim_signal::Connection &c);

protected:
    QBoxLayout *layout_;
    bool is_gui_setup_;

    std::string current_name_;
    std::string current_display_name_;
    QBoxLayout* current_layout_;

    NodeHandleWeakPtr node_;
    WidgetController* widget_ctrl_;

    std::vector<csapex::slim_signal::Connection> connections_;
};

}

#endif // NODE_ADAPTER_H

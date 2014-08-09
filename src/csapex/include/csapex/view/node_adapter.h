#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>

/// SYSTEM
#include <QLayout>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <vector>

class QListView;

namespace csapex
{
class NodeAdapter : public param::Parameter::access
{
public:
    typedef boost::shared_ptr<NodeAdapter> Ptr;

protected:
    NodeAdapter(Node* adaptee, WidgetController *widget_ctrl);

public:
    virtual ~NodeAdapter();

    Node* getNode();

    void doSetupUi(QBoxLayout* layout);

    virtual void nodeModelChangedEvent();
    virtual void updateDynamicGui(QBoxLayout* layout);

    virtual void stop();

    virtual MementoPtr getState() const;
    virtual void setParameterState(MementoPtr memento);

protected:
    virtual void setupUi(QBoxLayout* layout) = 0;

    virtual void guiChanged();

protected:
    QBoxLayout *layout_;
    bool is_gui_setup_;

    std::string current_name_;
    std::string current_display_name_;
    QBoxLayout* current_layout_;

    Node* node_;
    WidgetController* widget_ctrl_;
};

}

#endif // NODE_ADAPTER_H

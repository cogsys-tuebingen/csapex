/// HEADER
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace csapex;

NodeAdapter::NodeAdapter(Node *adaptee, WidgetController* widget_ctrl)
    : layout_(NULL), is_gui_setup_(false), node_(adaptee), widget_ctrl_(widget_ctrl)
{
}

NodeAdapter::~NodeAdapter()
{
}


Node* NodeAdapter::getNode()
{
    return node_;
}

void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    layout_ = layout;
    if(!is_gui_setup_) {

        try {
            setupUi(layout_);
            is_gui_setup_ = true;
            guiChanged();
        } catch(const std::exception& e) {
            std::cerr << "setting up ui for node " << node_->getUUID().getFullName() << " failed: " << e.what() << std::endl;
        }
    }
}

void NodeAdapter::nodeModelChangedEvent()
{

}

void NodeAdapter::updateDynamicGui(QBoxLayout* /*layout*/)
{

}

void NodeAdapter::stop()
{
}

void NodeAdapter::guiChanged()
{
}

Memento::Ptr NodeAdapter::getState() const
{
    Memento::Ptr state;
    return state;
}

void NodeAdapter::setParameterState(Memento::Ptr)
{

}

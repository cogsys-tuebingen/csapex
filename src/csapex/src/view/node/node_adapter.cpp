/// HEADER
#include <csapex/view/node/node_adapter.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/param/parameter.h>
#include <csapex/view/node/box.h>

/// SYSTEM
#include <QLayout>
#include <QWidget>
#include <iostream>

using namespace csapex;

NodeAdapter::NodeAdapter(NodeHandleWeakPtr adaptee, NodeBox* parent)
    : layout_(nullptr), is_gui_setup_(false), node_(adaptee), parent_(parent)
{
}

NodeAdapter::~NodeAdapter()
{
    for(auto& c : connections_) {
        c.disconnect();
    }

    connections_.clear();
}


void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    layout_ = layout;
    if(!is_gui_setup_) {

        try {
            setupUi(layout_);

            is_gui_setup_ = true;
        } catch(const std::exception& e) {
            std::cerr << "setting up ui for node " << node_.lock()->getUUID().getFullName() << " failed: " << e.what() << std::endl;
        }
    }
}

void NodeAdapter::invalidate()
{
    layout_->invalidate();
}

void NodeAdapter::stop()
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

void NodeAdapter::setManualResize(bool manual)
{

}

bool NodeAdapter::isResizable() const
{
    return false;
}

void NodeAdapter::trackConnection(const csapex::slim_signal::Connection &c)
{
    connections_.push_back(c);
}

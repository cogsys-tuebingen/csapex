/// HEADER
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace csapex;

NodeAdapter::NodeAdapter(Node *adaptee)
    : bridge(this), layout_(NULL), is_gui_setup_(false), node_(adaptee)
{
    QObject::connect(&bridge, SIGNAL(rebuild()), &bridge, SLOT(rebuildEvent()));
    QObject::connect(adaptee, SIGNAL(modelChanged()), &bridge, SLOT(modelChangedEvent()));
}

NodeAdapter::~NodeAdapter()
{
}

//void NodeAdapter::setNode(Node *node)
//{
//    node_ = node;

//    QObject::connect(node, SIGNAL(modelChanged()), &bridge, SLOT(modelChangedEvent()));
//}

Node* NodeAdapter::getNode()
{
    return node_;
}

void NodeAdapter::setupUiAgain()
{
    bridge.triggerRebuild();
}

void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    layout_ = layout;
    if(!is_gui_setup_) {
        is_gui_setup_ = true;

        try {
            setupUi(layout_);
            guiChanged();
        } catch(const std::exception& e) {
            std::cerr << "setting up ui for node " << node_->getUUID().getFullName() << " failed: " << e.what() << std::endl;
        }
    }
}


void NodeAdapter::modelChangedEvent()
{

}

void NodeAdapter::updateDynamicGui(QBoxLayout* /*layout*/)
{

}

void NodeAdapter::guiChanged()
{
    bridge.triggerGuiChanged();
}

NodeAdapterBridge::NodeAdapterBridge(NodeAdapter *parent)
    : parent_(parent)
{

}

void NodeAdapterBridge::modelChangedEvent()
{
    parent_->modelChangedEvent();
}

void NodeAdapterBridge::rebuildEvent()
{
    parent_->setupUi(parent_->layout_);
}

void NodeAdapterBridge::triggerGuiChanged()
{
    Q_EMIT guiChanged();
}

void NodeAdapterBridge::triggerRebuild()
{
    Q_EMIT rebuild();
}

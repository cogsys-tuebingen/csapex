/// HEADER
#include <csapex/view/node/node_adapter.h>

/// COMPONENT
#include <csapex/model/node_facade.h>
#include <csapex/param/parameter.h>
#include <csapex/view/node/box.h>

/// SYSTEM
#include <QLayout>
#include <QWidget>
#include <iostream>

using namespace csapex;

NodeAdapter::NodeAdapter(NodeFacadePtr adaptee, NodeBox* parent)
    : layout_(nullptr), is_gui_setup_(false), node_(adaptee), parent_(parent)
{
}

NodeAdapter::~NodeAdapter()
{
    stopObserving();
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

void NodeAdapter::readLegacyYaml(const YAML::Node & )
{

}

void NodeAdapter::setManualResize(bool manual)
{

}

bool NodeAdapter::isResizable() const
{
    return false;
}

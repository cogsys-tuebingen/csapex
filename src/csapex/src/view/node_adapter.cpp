/// HEADER
#include <csapex/view/node_adapter.h>

using namespace csapex;

NodeAdapter::~NodeAdapter()
{

}

void NodeAdapter::fill(QBoxLayout *)
{

}

void NodeAdapter::updateDynamicGui(QBoxLayout *)
{

}

void NodeAdapter::guiChanged()
{
    bridge.triggerGuiChanged();
}

void NodeAdapterBridge::triggerGuiChanged()
{
    Q_EMIT guiChanged();
}

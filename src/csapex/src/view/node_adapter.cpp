/// HEADER
#include <csapex/view/node_adapter.h>

using namespace csapex;

NodeAdapter::~NodeAdapter()
{

}

void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    setupUi(layout);

    guiChanged();
}

void NodeAdapter::setupUi(QBoxLayout *)
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

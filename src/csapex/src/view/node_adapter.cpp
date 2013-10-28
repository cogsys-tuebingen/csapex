/// HEADER
#include <csapex/view/node_adapter.h>

using namespace csapex;

NodeAdapter::NodeAdapter()
    : is_gui_setup_(false)
{

}

NodeAdapter::~NodeAdapter()
{

}

void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    if(!is_gui_setup_) {
        is_gui_setup_ = true;
        setupUi(layout);

        guiChanged();
    }
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

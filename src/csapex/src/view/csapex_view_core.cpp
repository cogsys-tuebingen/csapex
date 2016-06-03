/// HEADER
#include <csapex/view/csapex_view_core.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>

using namespace csapex;

CsApexViewCore::CsApexViewCore(CsApexCore& core, NodeAdapterFactory &node_adapter_factory, CommandDispatcher &dispatcher, DragIO &dragio)
    : core_(core), node_adapter_factory_(node_adapter_factory), dispatcher_(dispatcher),
      drag_io(dragio)
{

}

void CsApexViewCore::execute(const CommandPtr &command)
{
    dispatcher_.execute(command);
}
void CsApexViewCore::executeLater(const CommandPtr &command)
{
    dispatcher_.executeLater(command);
}

CsApexCore& CsApexViewCore::getCore()
{
    return core_;
}

NodeAdapterFactory& CsApexViewCore::getNodeAdapterFactory()
{
    return node_adapter_factory_;
}

CommandDispatcher& CsApexViewCore::getCommandDispatcher()
{
    return dispatcher_;
}

DesignerStyleable& CsApexViewCore::getStyle()
{
    return style;
}

DragIO& CsApexViewCore::getDragIO()
{
    return drag_io;
}

Settings& CsApexViewCore::getSettings()
{
    return core_.getSettings();
}

bool CsApexViewCore::isDebug() const
{
    return core_.getSettings().get<bool>("debug", false);
}

bool CsApexViewCore::isGridLockEnabled() const
{
    return core_.getSettings().get<bool>("grid-lock", false);
}

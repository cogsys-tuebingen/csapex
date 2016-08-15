#ifndef CSAPEX_VIEW_CORE_H
#define CSAPEX_VIEW_CORE_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/view/designer/designer_options.h>
#include <csapex/command/command_fwd.h>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcher;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class CSAPEX_QT_EXPORT CsApexViewCore
{
public:
    CsApexViewCore(CsApexCore& core, NodeAdapterFactory &node_adapter_factory, CommandDispatcher &dispatcher, DragIO& dragio);

    void execute(const CommandPtr& command);
    void executeLater(const CommandPtr& command);

    CsApexCore& getCore();

    NodeAdapterFactory& getNodeAdapterFactory();
    CommandDispatcher& getCommandDispatcher();

    DesignerStyleable& getStyle();

    DragIO& getDragIO();

    Settings& getSettings();

    bool isDebug() const;
    bool isGridLockEnabled() const;

private:
    CsApexCore& core_;

    NodeAdapterFactory& node_adapter_factory_;

    CommandDispatcher& dispatcher_;

    DesignerStyleable style;

    DragIO& drag_io;
};

}

#endif // CSAPEX_VIEW_CORE_H

#ifndef CSAPEX_VIEW_CORE_H
#define CSAPEX_VIEW_CORE_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/view/designer/designer_options.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/model/observer.h>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcher;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class CSAPEX_QT_EXPORT CsApexViewCore : public Observer
{
public:
    CsApexViewCore(CsApexCore& core);
    CsApexViewCore(CsApexViewCore& parent, CsApexCore& core, std::shared_ptr<CommandDispatcher> dispatcher);

    void execute(const CommandPtr& command);
    void executeLater(const CommandPtr& command);

    CsApexCore& getCore();

    std::shared_ptr<NodeAdapterFactory> getNodeAdapterFactory();
    CommandDispatcher& getCommandDispatcher();

    DesignerStyleable& getStyle();

    std::shared_ptr<DragIO> getDragIO();

    Settings& getSettings();

    bool isDebug() const;
    bool isGridLockEnabled() const;

private:
    CsApexCore& core_;

    std::shared_ptr<NodeAdapterFactory> node_adapter_factory_;
    std::shared_ptr<CommandDispatcher> dispatcher_;

    DesignerStyleable style;

    std::shared_ptr<DragIO> drag_io;
};

}

#endif // CSAPEX_VIEW_CORE_H

#ifndef CSAPEX_VIEW_CORE_H
#define CSAPEX_VIEW_CORE_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/view/designer/designer_options.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/utility/notifier.h>
#include <csapex/model/observer.h>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcher;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class CSAPEX_QT_EXPORT CsApexViewCore : public Observer, public Notifier
{
public:
    CsApexViewCore(CsApexCore& core);
    CsApexViewCore(CsApexViewCore& parent, CsApexCore& core, std::shared_ptr<CommandDispatcher> dispatcher);

    void execute(const CommandPtr& command);
    void executeLater(const CommandPtr& command);

    void setPause(bool paused);
    bool isPaused() const;

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();

    void stop();
    void clearBlock();
    void resetActivity();




    // TODO: remove
    CsApexCore& getCore();
    CommandDispatcher& getCommandDispatcher();

    // TODO: remove or proxy
    Settings& getSettings();




    std::shared_ptr<NodeAdapterFactory> getNodeAdapterFactory();

    DesignerStyleable& getStyle();

    std::shared_ptr<DragIO> getDragIO();

    bool isDebug() const;
    bool isGridLockEnabled() const;

public:
    slim_signal::Signal<void ()> config_changed;
    slim_signal::Signal<void (const std::string& msg)> status_changed;
    slim_signal::Signal<void ()> new_node_type;
    slim_signal::Signal<void ()> new_snippet_type;

    slim_signal::Signal<void ()> reset_requested;
    slim_signal::Signal<void ()> reset_done;

    slim_signal::Signal<void ()> saved;
    slim_signal::Signal<void ()> loaded;

    slim_signal::Signal<void (bool)> paused;

    slim_signal::Signal<void ()> begin_step;
    slim_signal::Signal<void ()> end_step;

    /// GRAPH
    slim_signal::Signal<void(NodeHandlePtr)> node_added;
    slim_signal::Signal<void(NodeHandlePtr)> node_removed;
    slim_signal::Signal<void(NodeWorkerPtr)> node_worker_added;
    slim_signal::Signal<void(NodeWorkerPtr)> node_worker_removed;
    slim_signal::Signal<void()> panic;


    /// THREAD POOL
    slim_signal::Signal<void (ThreadGroupPtr)> group_created;
    slim_signal::Signal<void (ThreadGroupPtr)> group_removed;

private:
    CsApexCore& core_;

    std::shared_ptr<NodeAdapterFactory> node_adapter_factory_;
    std::shared_ptr<CommandDispatcher> dispatcher_;

    DesignerStyleable style;

    std::shared_ptr<DragIO> drag_io;
};

}

#endif // CSAPEX_VIEW_CORE_H

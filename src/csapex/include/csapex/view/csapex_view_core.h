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
#include <csapex/command/command_executor.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/profiling/profiling_fwd.h>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcher;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class CSAPEX_QT_EXPORT CsApexViewCore :
        // abstract classes
        public Observer, public Notifier,
        // interfaces
        public CommandExecutor
{
public:
    CsApexViewCore(CsApexCore& core);
    CsApexViewCore(CsApexViewCore& parent, CsApexCore& core, std::shared_ptr<CommandDispatcher> dispatcher);

    // CORE
    void reset();
    void load(const std::string& file);
    void saveAs(const std::string& file, bool quiet = false);

    void setPause(bool paused);
    bool isPaused() const;

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();


    /// TODO: change to proxy?
    void execute(const CommandPtr& command);
    void executeLater(const CommandPtr& command);
    void executeLater();

    void undo();
    void redo();
    bool canUndo() const;
    bool canRedo() const;
    bool isDirty() const;

    Settings& getSettings();
    ExceptionHandler& getExceptionHandler() const;

    // TODO: add proxies
    GraphFacadePtr getRoot();
    void shutdown();
    void clearBlock();
    void resetActivity();

    ThreadPoolPtr getThreadPool();


    void sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level = ErrorState::ErrorLevel::ERROR);


    // TODO: only for direct sessions:
    CommandDispatcherPtr getCommandDispatcher();



    // TODO: remove
    CsApexCore& getCore();

    PluginLocatorPtr getPluginLocator() const;
    NodeFactory &getNodeFactory() const;
    SnippetFactory& getSnippetFactory() const;

    ProfilerPtr getProfiler() const;




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

    /// COMMANDS
    slim_signal::Signal<void ()> undo_state_changed;
    slim_signal::Signal<void (bool)> undo_dirty_changed;

private:
    CsApexCore& core_;

    std::shared_ptr<NodeAdapterFactory> node_adapter_factory_;
    std::shared_ptr<CommandDispatcher> dispatcher_;

    DesignerStyleable style;

    std::shared_ptr<DragIO> drag_io;

    ExceptionHandler &exception_handler_;
};

}

#endif // CSAPEX_VIEW_CORE_H

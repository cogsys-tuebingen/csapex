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

namespace YAML
{
class Node;
}

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
    CsApexViewCore();

    virtual void sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level = ErrorState::ErrorLevel::ERROR) = 0;

    // CORE
    virtual void reset() = 0;
    virtual void load(const std::string& file) = 0;
    virtual void saveAs(const std::string& file, bool quiet = false) = 0;

    virtual void setPause(bool paused) = 0;
    virtual bool isPaused() const = 0;

    virtual bool isSteppingMode() const = 0;
    virtual void setSteppingMode(bool stepping) = 0;
    virtual void step() = 0;

    virtual Settings& getSettings() const = 0;

    // TODO: add proxies or remove
    virtual ExceptionHandler& getExceptionHandler() const = 0;
    virtual GraphFacadePtr getRoot() = 0;
    virtual void shutdown() = 0;
    virtual void clearBlock() = 0;
    virtual void resetActivity() = 0;

    virtual ThreadPoolPtr getThreadPool() = 0;

    virtual CommandExecutorPtr getCommandDispatcher() = 0;

    virtual PluginLocatorPtr getPluginLocator() const = 0;
    virtual NodeFactoryPtr getNodeFactory() const = 0;
    virtual SnippetFactoryPtr getSnippetFactory() const = 0;

    virtual ProfilerPtr getProfiler() const = 0;

    virtual std::shared_ptr<DragIO> getDragIO() = 0;

    virtual std::shared_ptr<NodeAdapterFactory> getNodeAdapterFactory() = 0;



    DesignerStyleable& getStyle();


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

    csapex::slim_signal::Signal<void (SubgraphNodeConstPtr, YAML::Node& e)> save_detail_request;
    csapex::slim_signal::Signal<void (SubgraphNodePtr, const YAML::Node& n)> load_detail_request;

    /// GRAPH
    slim_signal::Signal<void(NodeFacadePtr)> node_facade_added;
    slim_signal::Signal<void(NodeFacadePtr)> node_facade_removed;
    slim_signal::Signal<void()> panic;


    /// THREAD POOL
    slim_signal::Signal<void (ThreadGroupPtr)> group_created;
    slim_signal::Signal<void (ThreadGroupPtr)> group_removed;

    /// COMMANDS
    slim_signal::Signal<void ()> undo_state_changed;
    slim_signal::Signal<void (bool)> undo_dirty_changed;

private:
    DesignerStyleable style;
};

}

#endif // CSAPEX_VIEW_CORE_H

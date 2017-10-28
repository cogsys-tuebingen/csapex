#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
#include <csapex/csapex_export.h>
#include <csapex/model/observer.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/utility/notifier.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <thread>
#include <mutex>
#include <condition_variable>

namespace class_loader {
class ClassLoader;
}

namespace csapex
{

class Profiler;

class CSAPEX_EXPORT CsApexCore : public Observer, public Notifier
{
public:
    CsApexCore(Settings& settings_, ExceptionHandler &handler);
    CsApexCore(Settings& settings_, ExceptionHandler &handler, PluginLocatorPtr plugin_locator, NodeFactoryPtr node_factory, SnippetFactoryPtr snippet_factory);

    virtual ~CsApexCore();

    void init(bool create_global_ports = true);
    void boot();
    void startup();
    void shutdown();

    void startMainLoop();

    void load(const std::string& file);
    void saveAs(const std::string& file, bool quiet = false);

    SnippetPtr serializeNodes(const AUUID &graph_id, const std::vector<UUID>& nodes) const;

    void reset();

    Settings& getSettings() const;
    NodeFactoryPtr getNodeFactory() const;
    SnippetFactoryPtr getSnippetFactory() const;

    GraphFacadeLocalPtr getRoot() const;
    NodeFacadeLocalPtr getRootNode() const;
    ThreadPoolPtr getThreadPool() const;

    PluginLocatorPtr getPluginLocator() const;
    ExceptionHandler& getExceptionHandler() const;

    CommandDispatcherPtr getCommandDispatcher() const;

    std::shared_ptr<Profiler> getProfiler() const;

    bool isPaused() const;
    void setPause(bool pause);

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();

    void drainPipeline();

    void settingsChanged();
    void setStatusMessage(const std::string& msg);

    void sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level = ErrorState::ErrorLevel::ERROR);

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

    slim_signal::Signal<void ()> stepping_enabled;
    slim_signal::Signal<void ()> begin_step;
    slim_signal::Signal<void ()> end_step;

    slim_signal::Signal<void ()> startup_requested;
    slim_signal::Signal<void ()> shutdown_requested;
    slim_signal::Signal<void ()> shutdown_complete;

    // TODO: refactor this to support remote clients
    csapex::slim_signal::Signal<void (const GraphFacade&, YAML::Node& e)> save_detail_request;
    csapex::slim_signal::Signal<void (GraphFacade&, const YAML::Node& n)> load_detail_request;
    
private:
    CsApexCore(Settings& settings_, ExceptionHandler &handler, PluginLocatorPtr plugin_locator);
    CorePluginPtr makeCorePlugin(const std::string& name);

private:
    bool is_root_;

    BootstrapPtr bootstrap_;

    Settings& settings_;
    PluginLocatorPtr plugin_locator_;
    ExceptionHandler &exception_handler_;

    NodeFactoryPtr node_factory_;
    SnippetFactoryPtr snippet_factory_;

    ThreadPoolPtr thread_pool_;

    UUIDProviderPtr root_uuid_provider_;
    GraphFacadeLocalPtr root_;
    NodeFacadeLocalPtr root_facade_;
    NodeWorkerPtr root_worker_;
    TaskGeneratorPtr root_scheduler_;

    std::shared_ptr<CommandDispatcher> dispatcher_;

    std::shared_ptr<Profiler> profiler_;

    std::shared_ptr<PluginManager<CorePlugin>> core_plugin_manager;
    std::map<std::string, std::shared_ptr<CorePlugin> > core_plugins_;
    std::map<std::string, bool> core_plugins_connected_;

    std::thread main_thread_;
    std::mutex running_mutex_;
    std::condition_variable running_changed_;
    bool running_;

    bool init_;
    bool load_needs_reset_;
};

}

#endif // CSAPEX_CORE_H

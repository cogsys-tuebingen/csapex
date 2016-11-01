#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
#include <csapex/utility/uuid.h>
#include <csapex/csapex_export.h>
#include <csapex/utility/notification.h>
#include <csapex/utility/slim_signal.h>

namespace class_loader {
class ClassLoader;
}

namespace csapex
{

class Profiler;

class CSAPEX_EXPORT CsApexCore
{
public:
    CsApexCore(Settings& settings_, ExceptionHandler &handler);
    CsApexCore(const CsApexCore& parent);

    virtual ~CsApexCore();

    void init();
    void boot();
    void startup();

    void load(const std::string& file);
    void saveAs(const std::string& file, bool quiet = false);

    void reset();

    Settings& getSettings() const;
    NodeFactory& getNodeFactory() const;
    SnippetFactory& getSnippetFactory() const;

    GraphFacadePtr getRoot() const;
    ThreadPoolPtr getThreadPool() const;

    PluginLocatorPtr getPluginLocator() const;
    ExceptionHandler& getExceptionHandler() const;

    std::shared_ptr<Profiler> getProfiler() const;

    bool isPaused() const;
    void setPause(bool pause);

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();

    void settingsChanged();
    void setStatusMessage(const std::string& msg);

public:
    csapex::slim_signal::Signal<void ()> configChanged;
    csapex::slim_signal::Signal<void (const std::string& msg)> showStatusMessage;
    csapex::slim_signal::Signal<void ()> newNodeType;
    csapex::slim_signal::Signal<void ()> newSnippetType;

    csapex::slim_signal::Signal<void ()> resetRequest;
    csapex::slim_signal::Signal<void ()> resetDone;

    csapex::slim_signal::Signal<void ()> saved;
    csapex::slim_signal::Signal<void ()> loaded;

    csapex::slim_signal::Signal<void (bool)> paused;

    csapex::slim_signal::Signal<void (Notification)> notification;

    csapex::slim_signal::Signal<void ()> begin_step;
    csapex::slim_signal::Signal<void ()> end_step;

private:
    CsApexCore(Settings& settings_, ExceptionHandler &handler, csapex::PluginLocatorPtr plugin_locator);

    CorePluginPtr makeCorePlugin(const std::string& name);

private:
    const CsApexCore* parent_;

    Settings& settings_;
    csapex::PluginLocatorPtr plugin_locator_;
    ExceptionHandler &exception_handler_;

    NodeFactoryPtr node_factory_;
    SnippetFactoryPtr snippet_factory_;

    ThreadPoolPtr thread_pool_;

    std::shared_ptr<UUIDProvider> root_uuid_provider_;
    GraphFacadePtr root_;
    NodeHandlePtr root_handle_;
    NodeWorkerPtr root_worker_;
    TaskGeneratorPtr root_scheduler_;

    std::shared_ptr<Profiler> profiler_;

    std::vector<slim_signal::ScopedConnection> signal_connections_;

    std::shared_ptr<PluginManager<CorePlugin>> core_plugin_manager;
    std::map<std::string, std::shared_ptr<CorePlugin> > core_plugins_;
    std::map<std::string, bool> core_plugins_connected_;

    std::vector<BootstrapPluginPtr> boot_plugins_;
    std::vector<class_loader::ClassLoader*> boot_plugin_loaders_;

    bool init_;
    bool load_needs_reset_;
};

}

#endif // CSAPEX_CORE_H

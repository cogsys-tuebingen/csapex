#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
#include <csapex/command/meta.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>
#include <yaml-cpp/yaml.h>

namespace class_loader {
class ClassLoader;
}

namespace csapex
{

class CsApexCore
{
public:
    CsApexCore(Settings& settings_,
               PluginLocatorPtr plugin_locator,
               GraphWorkerPtr graph, ThreadPool& thread_pool,
               NodeFactory* node_factory, NodeAdapterFactory* node_adapter_factory,
               CommandDispatcher *cmd_dispatcher);
    virtual ~CsApexCore();

    void init(DragIO *dragio);
    void boot();
    void startup();

    void load(const std::string& file);
    void saveAs(const std::string& file);

    void reset();

    void unloadNode(csapex::UUID uuid);
    void reloadDone();

    Settings& getSettings() const;
    NodeFactory& getNodeFactory() const;

    bool isPaused() const;
    void setPause(bool pause);

    void settingsChanged();
    void setStatusMessage(const std::string& msg);

public:
    boost::signals2::signal<void ()> configChanged;
    boost::signals2::signal<void (const std::string& msg)> showStatusMessage;
    boost::signals2::signal<void ()> newNodeType;

    boost::signals2::signal<void ()> resetRequest;
    boost::signals2::signal<void ()> resetDone;

    boost::signals2::signal<void (YAML::Node& e)> saveSettingsRequest;
    boost::signals2::signal<void (YAML::Node& n)> loadSettingsRequest;

    boost::signals2::signal<void (YAML::Node& e)> saveViewRequest;
    boost::signals2::signal<void (YAML::Node& n)> loadViewRequest;

    boost::signals2::signal<void (bool)> paused;

private:
    CorePluginPtr makeCorePlugin(const std::string& name);
    void unloadCorePlugin(const std::string& plugin);
    void reloadCorePlugin(const std::string& plugin);

private:
    Settings& settings_;
    DragIO* drag_io_;

    csapex::PluginLocatorPtr plugin_locator_;

    GraphWorkerPtr graph_worker_;
    ThreadPool& thread_pool_;

    NodeFactory* node_factory_;
    NodeAdapterFactory* node_adapter_factory_;

    bool destruct;
    CommandDispatcher* cmd_dispatch;

    PluginManager<CorePlugin>* core_plugin_manager;
    std::map<std::string, std::shared_ptr<CorePlugin> > core_plugins_;
    std::map<std::string, bool> core_plugins_connected_;

    std::vector<BootstrapPluginPtr> boot_plugins_;
    std::vector<class_loader::ClassLoader*> boot_plugin_loaders_;

    command::Meta::Ptr unload_commands_;

    bool init_;
};

}

#endif // CSAPEX_CORE_H

#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
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
               GraphWorkerPtr graph_worker, GraphPtr graph,
               ThreadPool& thread_pool,
               NodeFactory* node_factory);
    virtual ~CsApexCore();

    void init();
    void boot();
    void startup();

    void load(const std::string& file);
    void saveAs(const std::string& file);

    void reset();

    Settings& getSettings() const;
    NodeFactory& getNodeFactory() const;

    bool isPaused() const;
    void setPause(bool pause);

    bool isSteppingMode() const;
    void setSteppingMode(bool stepping);
    void step();

    void settingsChanged();
    void setStatusMessage(const std::string& msg);

public:
    boost::signals2::signal<void ()> configChanged;
    boost::signals2::signal<void (const std::string& msg)> showStatusMessage;
    boost::signals2::signal<void ()> newNodeType;

    boost::signals2::signal<void ()> resetRequest;
    boost::signals2::signal<void ()> resetDone;

    boost::signals2::signal<void ()> saved;
    boost::signals2::signal<void ()> loaded;

    boost::signals2::signal<void (YAML::Node& e)> saveSettingsRequest;
    boost::signals2::signal<void (YAML::Node& n)> loadSettingsRequest;

    boost::signals2::signal<void (YAML::Node& e)> saveViewRequest;
    boost::signals2::signal<void (YAML::Node& n)> loadViewRequest;

    boost::signals2::signal<void (bool)> paused;

    boost::signals2::signal<void ()> begin_step;
    boost::signals2::signal<void ()> end_step;

private:
    CorePluginPtr makeCorePlugin(const std::string& name);
    void unloadCorePlugin(const std::string& plugin);
    void reloadCorePlugin(const std::string& plugin);

private:
    Settings& settings_;

    csapex::PluginLocatorPtr plugin_locator_;

    GraphWorkerPtr graph_worker_;
    GraphPtr graph_;
    ThreadPool& thread_pool_;

    NodeFactory* node_factory_;

    bool destruct;

    PluginManager<CorePlugin>* core_plugin_manager;
    std::map<std::string, std::shared_ptr<CorePlugin> > core_plugins_;
    std::map<std::string, bool> core_plugins_connected_;

    std::vector<BootstrapPluginPtr> boot_plugins_;
    std::vector<class_loader::ClassLoader*> boot_plugin_loaders_;

    bool init_;
};

}

#endif // CSAPEX_CORE_H

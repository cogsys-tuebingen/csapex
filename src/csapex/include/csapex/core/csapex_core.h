#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <csapex/utility/slim_signal.h>

namespace class_loader {
class ClassLoader;
}

namespace csapex
{

class CsApexCore
{
public:
    CsApexCore(Settings& settings_,
               PluginLocatorPtr plugin_locator, GraphPtr graph,
               ThreadPool& thread_pool,
               NodeFactory* node_factory);
    virtual ~CsApexCore();

    void init();
    void boot();
    void startup();

    void load(const std::string& file);
    void saveAs(const std::string& file, bool quiet = false);

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
    csapex::slim_signal::Signal<void ()> configChanged;
    csapex::slim_signal::Signal<void (const std::string& msg)> showStatusMessage;
    csapex::slim_signal::Signal<void ()> newNodeType;

    csapex::slim_signal::Signal<void ()> resetRequest;
    csapex::slim_signal::Signal<void ()> resetDone;

    csapex::slim_signal::Signal<void ()> saved;
    csapex::slim_signal::Signal<void ()> loaded;

    csapex::slim_signal::Signal<void (bool)> paused;

    csapex::slim_signal::Signal<void ()> begin_step;
    csapex::slim_signal::Signal<void ()> end_step;

private:
    CorePluginPtr makeCorePlugin(const std::string& name);

private:
    Settings& settings_;

    csapex::PluginLocatorPtr plugin_locator_;

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
    bool load_needs_reset_;
};

}

#endif // CSAPEX_CORE_H

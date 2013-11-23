#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/command/dispatcher.h>

/// SYSTEM
#include <QObject>
#include <yaml-cpp/yaml.h>

namespace csapex
{

class CsApexCore : public QObject
{
    Q_OBJECT

public:
    struct Listener {
        virtual void resetSignal() = 0;
    };

public:
    CsApexCore(const std::string &config);
    CsApexCore(CommandDispatcher* dispatcher);
    virtual ~CsApexCore();

    void init();

    std::string getConfig() const;
    void setCurrentConfig(const std::string& filename);

    void load(const std::string& file);
    void saveAs(const std::string& file);

    void reset();

    void addListener(Listener* l);
    void removeListener(Listener* l);

    GraphPtr getTopLevelGraph();
    CommandDispatcher* getCommandDispatcher();

Q_SIGNALS:
    void configChanged();
    void showStatusMessage(const std::string& msg);
    void reloadBoxMenues();

    void saveSettingsRequest(YAML::Emitter& e);
    void loadSettingsRequest(YAML::Node& n);

private:
    bool destruct;
    CommandDispatcher* cmd_dispatch;

    PluginManager<CorePlugin>* core_plugin_manager;
    std::vector<Listener*> listener_;

    std::string current_config_;

    bool init_;
};

}

#endif // CSAPEX_CORE_H

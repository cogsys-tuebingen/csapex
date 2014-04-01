#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/settings.h>

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
    CsApexCore(Settings& settings_, GraphPtr graph, CommandDispatcher *cmd_dispatcher);
    virtual ~CsApexCore();

    void init(DragIO *dragio);

    void load(const std::string& file);
    void saveAs(const std::string& file);

    void reset();

    void addListener(Listener* l);
    void removeListener(Listener* l);

    Settings& getSettings() const;

    bool isPaused() const;

public Q_SLOTS:
    void setPause(bool pause);
    void settingsChanged();

Q_SIGNALS:
    void configChanged();
    void showStatusMessage(const std::string& msg);
    void reloadBoxMenues();

    void saveSettingsRequest(YAML::Emitter& e);
    void loadSettingsRequest(YAML::Node& n);

    void paused(bool);

private:
    Settings& settings_;
    GraphPtr graph_;

    bool destruct;
    CommandDispatcher* cmd_dispatch;

    PluginManager<CorePlugin>* core_plugin_manager;
    std::vector<boost::shared_ptr<CorePlugin> > core_plugins_;
    std::vector<Listener*> listener_;

    bool init_;
};

}

#endif // CSAPEX_CORE_H

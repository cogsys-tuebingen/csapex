#ifndef CSAPEX_CORE_H
#define CSAPEX_CORE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <QObject>
#include <yaml-cpp/yaml.h>

namespace csapex
{

class CsApexCore : public QObject
{
    Q_OBJECT

public:
    CsApexCore();
    virtual ~CsApexCore();

    void init();

    std::string getConfig() const;
    void setCurrentConfig(const std::string& filename);

    void load(const std::string& file);
    void saveAs(const std::string& file);

Q_SIGNALS:
    void configChanged();
    void showStatusMessage(const std::string& msg);

    void saveSettingsRequest(YAML::Emitter& e);
    void loadSettingsRequest(YAML::Node& n);

private:
    PluginManager<CorePlugin> core_plugin_manager;

    std::string current_config_;

    bool init_;
};

}

#endif // CSAPEX_CORE_H

#ifndef SETTINGS_REMOTE_H
#define SETTINGS_REMOTE_H

/// COMPONENT
#include <csapex_remote_export.h>
#include <csapex/core/settings.h>
#include <csapex/io/io_fwd.h>
#include <csapex/io/remote.h>

namespace csapex
{

class CSAPEX_REMOTE_EXPORT SettingsRemote : public Settings, public Remote
{
public:
    SettingsRemote(const SessionPtr &session);

    bool knows(const std::string& name) const override;

    void add(csapex::param::Parameter::Ptr p, bool persistent) override;
    param::ParameterPtr get(const std::string& name) const override;
    param::ParameterPtr getNoThrow(const std::string& name) const override;

    void savePersistent() override;
    void loadPersistent() override;

    void saveTemporary(YAML::Node& node) override;
    void loadTemporary(YAML::Node& node) override;

private:
    void createParameterProxy(const std::string &name, param::ParameterPtr proxy) const;

    void handleBroadcast(const BroadcastMessageConstPtr& message) override;

private:
    mutable std::map<std::string, param::ParameterPtr> cache_;
};

}

#endif // SETTINGS_REMOTE_H

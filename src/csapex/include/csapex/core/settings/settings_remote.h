#ifndef SETTINGS_REMOTE_H
#define SETTINGS_REMOTE_H

/// COMPONENT
#include <csapex_remote_export.h>
#include <csapex/core/settings.h>
#include <csapex/io/io_fwd.h>

namespace csapex
{

class CSAPEX_REMOTE_EXPORT SettingsRemote : public Settings
{
public:
    SettingsRemote(SessionPtr session);

    bool knows(const std::string& name) const override;

    void add(csapex::param::Parameter::Ptr p, bool persistent) override;
    param::ParameterPtr get(const std::string& name) const override;
    param::ParameterPtr getNoThrow(const std::string& name) const override;

    void save() override;
    void load() override;

private:
    void createParameterProxy(const std::string &name, param::ParameterPtr proxy) const;

private:
    SessionPtr session_;

    mutable std::map<std::string, param::ParameterPtr> cache_;
};

}

#endif // SETTINGS_REMOTE_H

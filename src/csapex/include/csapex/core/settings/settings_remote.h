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

    bool isQuiet() const;
    void setQuiet(bool quiet);

    bool knows(const std::string& name) const;

    void add(csapex::param::Parameter::Ptr p, bool persistent);
    param::ParameterPtr get(const std::string& name) const;
    param::ParameterPtr getNoThrow(const std::string& name) const;

    void addTemporary(csapex::param::Parameter::Ptr p);
    void addPersistent(csapex::param::Parameter::Ptr p);

    void save();
    void load();

private:
    SessionPtr session_;

    mutable std::map<std::string, param::ParameterPtr> cache_;
};

}

#endif // SETTINGS_REMOTE_H

#ifndef GENERIC_STATE_H
#define GENERIC_STATE_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/utility/uuid.h>
#include <csapex/csapex_export.h>

/// PROJECT
#include <csapex/param/param_fwd.h>

/// SYSTEM
#include <csapex/utility/slim_signal.h>
#include <set>

namespace csapex
{

class CSAPEX_EXPORT GenericState : public Memento
{
public:
    typedef std::shared_ptr<GenericState> Ptr;

public:
    GenericState();
    GenericState::Ptr clone() const;

    void setParentUUID(const UUID& parent_uuid);

    void addParameter(csapex::param::ParameterPtr param);
    void removeParameter(csapex::param::ParameterPtr param);

    void addPersistentParameter(const csapex::param::ParameterPtr& param);

    void addTemporaryParameter(const csapex::param::ParameterPtr& param);
    void removeTemporaryParameter(const csapex::param::ParameterPtr& param);
    void removeTemporaryParameters();

    void setParameterSetSilence(bool silent);
    void triggerParameterSetChanged();

    csapex::param::Parameter& operator [] (const std::string& name) const;
    csapex::param::ParameterPtr getParameter(const std::string& name) const;
    csapex::param::ParameterPtr getMappedParameter(const std::string& name) const;
    std::vector<csapex::param::ParameterPtr> getParameters() const;
    std::vector<csapex::param::ParameterPtr> getTemporaryParameters() const;
    std::vector<csapex::param::ParameterPtr> getPersistentParameters() const;

    std::size_t getParameterCount() const;
    bool hasParameter(const std::string& name) const;

    template <typename T>
    T readParameter (const std::string& name) const;

    void setFrom(const GenericState& rhs);

    virtual void writeYaml(YAML::Node& out) const override;
    virtual void readYaml(const YAML::Node& node) override;

    void initializePersistentParameters();

private:
    void registerParameter(const csapex::param::ParameterPtr &param);
    void unregisterParameter(const csapex::param::ParameterPtr &param);

public:
    UUID parent_uuid_;

    std::map<std::string, csapex::param::ParameterPtr> params;
    std::map<std::string, std::string> param_valid_name_cache;
    std::map<std::string, bool> temporary;
    std::set<std::string> persistent;
    std::set<std::string> legacy;
    std::vector<std::string> order;

    bool silent_;

    std::shared_ptr<slim_signal::Signal<void()> > parameter_set_changed;
    std::shared_ptr<slim_signal::Signal<void(csapex::param::ParameterPtr)> > parameter_added;
    std::shared_ptr<slim_signal::Signal<void(csapex::param::ParameterPtr)> > parameter_removed;
};

}

#endif // GENERIC_STATE_H
